#include "llvm/BasicBlock.h"
#include "llvm/IntrinsicInst.h"
#include "llvm/Module.h"
#include "llvm/Pass.h"
#include "llvm/Support/CallSite.h"

#include "LocalFrequency.h"

#include <map>

using namespace llvm;
using namespace SeansLLVM;

class LocalCallFrequency : public FunctionPass
{
protected:
  typedef std::map<Function *, float> FrequencyMapTy;
  FrequencyMapTy CalleeFrequencies;

public:
  static char ID;
  typedef FrequencyMapTy::iterator iterator;

  LocalCallFrequency() : FunctionPass(ID) {}

  virtual bool runOnFunction(Function &F)
  {
    LocalFrequencies &LBF = getAnalysis<LocalFrequencies>();
    for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I)
    {
      for (BasicBlock::iterator BBI = I->begin(), BBE = I->end();
           BBI != BBE; ++BBI)
      {
        CallSite CS(cast<Value>(BBI));
        if (CS && !isa<DbgInfoIntrinsic>(BBI))
        {
          Function *Callee = CS.getCalledFunction();
          if (Callee)
          {
            CalleeFrequencies[Callee] += LBF[*I];
          }
        }
      }
    }

    return false;
  }

  iterator begin() { return CalleeFrequencies.begin(); }
  iterator end() { return CalleeFrequencies.end(); }

};

char LocalCallFrequency::ID = 0;

class GlobalFrequencies : public ModulePass
{
public:
  static char ID;
  GlobalFrequencies() : ModulePass(ID) {};
  virtual bool runOnModule(Module &M);

protected:
  typedef std::pair<Function *, Function *> Edge;

  void init(Function *root);
  void UnmarkReachable(Function *F);
  bool isVisited(Function *f);
  void PropagateCallFrequencies(Function *f, Function *head, bool isMain);

  std::vector<Function *> DepthFirstOrder;
  std::set<Function *> LoopHeads;
  std::map<Edge, float> BackEdgeProbability;
  std::set<Function *> ToVisit;
  std::map<Function *, std::set<Function *> > Predecessors;
  std::set<Edge> BackEdges;
  std::map<Function *, float> CallFrequency;
  std::map<Edge, float> GlobalEdgeFrequency;
};

char GlobalFrequencies::ID = 0;

void GlobalFrequencies::init(Function *root)
{
  std::set<Function *> Visited;
  std::vector<Function *> Stack;

  Stack.push_back(root);
  Visited.insert(root);
  while (!Stack.empty())
  {
    Function *caller = Stack.back();
    Stack.pop_back();
    DepthFirstOrder.push_back(caller);

    LocalCallFrequency &LCF = getAnalysis<LocalCallFrequency>(*caller);
    for (LocalCallFrequency::iterator I = LCF.begin(), E = LCF.end();
         I != E; ++I)
    {
      Function *callee = I->first;
      Edge edge = std::make_pair(caller, callee);

      Predecessors[callee].insert(caller);
      BackEdgeProbability[edge] = I->second;

      if (Visited.insert(callee).second)
      {
        Stack.push_back(callee);
      }
      else
      {
        LoopHeads.insert(callee);
        BackEdges.insert(edge);
      }
    }
  }
}

void GlobalFrequencies::UnmarkReachable(Function *F)
{
  std::vector<Function *> Stack;

  ToVisit.clear();
  ToVisit.insert(F);
  Stack.push_back(F);
  while (!Stack.empty())
  {
    Function *F = Stack.back();
    Stack.pop_back();
    LocalCallFrequency &LCF = getAnalysis<LocalCallFrequency>(*F);
    for (LocalCallFrequency::iterator I = LCF.begin(), E = LCF.end();
         I != E; ++I)
    {
      Function *SuccessorFunction = I->first;
      if (ToVisit.insert(SuccessorFunction).second)
      {
        Stack.push_back(SuccessorFunction);
      }
    }
  }
}

bool GlobalFrequencies::isVisited(Function *f)
{
  // visited if not found in ToVisit
  return (ToVisit.find(f) == ToVisit.end());
}

void GlobalFrequencies::PropagateCallFrequencies(Function *f, Function *head, bool isMain)
{
  if (isVisited(f))
  {
    return;
  }

  for (std::set<Function *>::iterator I = Predecessors[f].begin(), E = Predecessors[f].end();
       I != E; ++I)
  {
    Function *predecessor = *I;
    if (!isVisited(*I) && (BackEdges.count(std::make_pair(predecessor, f)) == 0))
    {
      return;
    }
  }

  if (f == head)
  {
    CallFrequency[f] = 1.0f;
  }
  else
  {
    CallFrequency[f] = 0.0f;
  }

  float cyclicProbability = 0;
  for (std::set<Function *>::iterator I = Predecessors[f].begin(), E = Predecessors[f].end();
       I != E; ++I)
  {
    Function *predecessor = *I;
    Edge edge(predecessor, f);
    if (isMain && BackEdges.count(edge))
    {
      cyclicProbability += BackEdgeProbability[edge];
    }
    else if (BackEdges.count(edge) == 0)
    {
      CallFrequency[f] += GlobalEdgeFrequency[edge];
    }
  }

  if (cyclicProbability > (1.0f - EPSILON))
  {
    cyclicProbability = 1.0f - EPSILON;
  }
  CallFrequency[f] /= (1.0f - cyclicProbability);

  ToVisit.erase(f);

  LocalCallFrequency &LCF = getAnalysis<LocalCallFrequency>(*f);
  for (LocalCallFrequency::iterator I = LCF.begin(), E = LCF.end();
       I != E; ++I)
  {
    Function *SuccessorFunction = I->first;
    float localEdgeFrequency = I->second;
    Edge SuccessorEdge(f, SuccessorFunction);
    GlobalEdgeFrequency[SuccessorEdge] = localEdgeFrequency * CallFrequency[f];

    if (!isMain && (SuccessorFunction == head))
    {
      BackEdgeProbability[SuccessorEdge] = localEdgeFrequency * CallFrequency[f];
    }
  }

  for (LocalCallFrequency::iterator I = LCF.begin(), E = LCF.end();
       I != E; ++I)
  {
    Edge SuccessorEdge(f, I->first);
    if (BackEdges.count(SuccessorEdge) == 0)
    {
      PropagateCallFrequencies(I->first, head, isMain);
    }
  }
}

bool GlobalFrequencies::runOnModule(Module &M)
{
  Function *root = M.getFunction("main");
  assert(root && "no main");

  init(root);

  for (std::vector<Function *>::reverse_iterator I = DepthFirstOrder.rbegin(), E = DepthFirstOrder.rend();
       I != E; ++I)
  {
    if (LoopHeads.find(*I) != LoopHeads.end())
    {
      UnmarkReachable(*I);
      PropagateCallFrequencies(*I, *I, false);
    }
  }

  UnmarkReachable(root);
  PropagateCallFrequencies(root, root, true);

  return false;
}

