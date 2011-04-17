#include "llvm/BasicBlock.h"
#include "llvm/IntrinsicInst.h"
#include "llvm/Pass.h"
#include "llvm/Support/CallSite.h"

#include <map>

using namespace llvm;

class BBCalls : public BasicBlockPass
{
protected:
  typedef std::map<const Function *, int> CalleeMap;
  CalleeMap Callees;

public:
  typedef CalleeMap::iterator iterator;

  virtual bool runOnBasicBlock(BasicBlock &BB)
  {
    for (BasicBlock::iterator I = BB.begin(), E = BB.end(); I != E; ++I)
    {
      CallSite CS(cast<Value>(I));
      if (CS && !isa<DbgInfoIntrinsic>(I))
      {
        const Function *Callee = CS.getCalledFunction();
        if (Callee)
        {
          Callees[Callee]++;
          assert(Callees[Callee] > 0 && "integer overflow");
        }
      }
    }
  }

  iterator begin() { return Callees.begin(); }
  iterator end() { return Callees.end(); }
};

class LocalCallFrequency : public FunctionPass
{
protected:
  std::map<const Function *, float> CalleeFrequencies;

public:
  virtual bool runOnFunction(Function &F)
  {
    for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I)
    {
      BBCalls &BBC = getAnalysis<BBCalls>(*I);
      for (BBCalls::iterator FI = BBC.begin(), FE = BBC.end(); I != E; ++I)
      {
        CalleeFrequencies[FI->first] += FI->second * LocalBlockFrequency[BB];
      }
    }
  }

};

class GlobalFrequencies : public ModulePass
{
public:
  static char ID;
  GlobalFrequencies();
  virtual bool runOnModule(Module &M);

protected:
  typedef std::pair<const Function *, const Function *> Edge;

  std::vector<const Function *> DepthFirstOrder;
  std::set<const Function *> LoopHeads;
  std::map<Edge, float> BackEdgeProbability;
  std::set<const Function *> toVisit;
  std::map<const Function *, std::set<const Function *>> Predecessors;
}

bool GlobalFrequencies::init()
{
  std::set<const Function *> Visited;
  std::vector<const Function *> Stack;

  Stack.push_back(root);
  Visited.insert(root);
  while (!Stack.empty())
  { 
    Function *caller = Stack.back();
    Stack.pop_back();
    DepthFirstOrder.push_back(caller);

    LocalCallFrequencies &LCF = getAnalysis<LocalCallFrequencies>(root);
    for (LocalCallFrequencies::iterator I = LCF.begin(), E = LCF.end(); 
         I != E; ++I)
    { 
      const Function *callee = I->first;
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

void GlobalFrequencies::UnmarkReachable(const Function *F)
{
  std::vector<const Function *> Stack;

  toVisit.clear();
  toVisit.insert(F);
  Stack.push_bacK(F);
  while (!Stack.empty())
  {
    const Function *F = Stack.back();
    Stack.pop_back();
    LocalCallFrequencies &LCF = getAnalysis<LocalCallFrequencies>(F);
    for (LocalCallFrequencies::iterator I = LCF.begin(), E = LCF.end();
         I != E; ++I)
    {
      const Funtion *SucessorFunction = I->first;
      if (toVisit.insert(SuccessorFunction).second)
      {
        Stack.push_back(SuccessorFunction);
      }
    }
  }
}

void GlobalFrequencies::isVisited(const Function *f)
{
  // visited if not found in toVisit
  return (toVisit.find(f) == toVisit.end())
}

void GlobalFrequencies::PropagateCallFrequencies(const Function *f, const Function *head, bool isMain)
{
  if (isVisited(f))
  {
    return;
  }

  for (std::set<const Function *>::iterator I = Predecessors[f].begin(), E = Predecessors[f].end();
       I != E; ++I)
  {
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
  for (std::set<const Function *>::iterator I = Predecessors[f].begin(), E = Predecessors[f].end();
       I != E; ++I)
  {
    const Function *predecessor = *I;
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

  if (cyclicProbability > (1.0f - epsilon))
  {
    cyclicProbability = 1.0f - epsilon;
  }
  CallFrequency[f] /= (1.0f - cyclicProbability);

  ToVisit.erase(f);

  LocalCallFrequencies &LCF = getAnalysis<LocalCallFrequencies>(F);
  for (LocalCallFrequencies::iterator I = LCF.begin(), E = LCF.end();
       I != E; ++I)
  {
    const Function *SuccessorFunction = I->first;
    float localEdgeFrequency = I->second;
    Edge SuccessorEdge(f, SuccessorFunction);
    GlobalEdgeFrequency[SuccessorEdge] = localEdgeFrequency * CallFrequency[f];

    if (!isMain && (SuccessorFunction == head))
    {
      BackEdgeProbability[SuccessorEdge] = localEdgeFrequency * CallFrequency[f];
    }
  }

  for (LocalCallFrequencies::iterator I = LCF.begin(), E = LCF.end();
       I != E; ++I)
  {
    Edge SuccessorEdge(f, I->first);
    if (BackEdges.count(SuccessorEdge) == 0)
    {
      propagateCallFrequencies(I->first, head, isMain);
    }
  }
}

bool GlobalFrequencies::runOnModule(Module &M) {
{
  Function *root = M.getFunction("main");
  assert(root && "no main");

  init(root);

  for (std::vector<const Function *>::iterator I = DepthFirstOrder.rbegin(), E = DepthFirstOrder.rend();
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
}

