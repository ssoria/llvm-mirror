#include <llvm/Function.h>
#include <llvm/Analysis/LoopInfo.h>
#include <llvm/Support/Debug.h>

#include "SeansBranchProbabilities.h"

namespace llvm {

#define EPSILON 0.05f

class LocalFrequencies : public FunctionPass
{
public:
  static char ID;
  LocalFrequencies();
  virtual bool runOnFunction(Function &F);

protected:
  typedef std::pair<BasicBlock *, BasicBlock *> Edge;
  void processLoop(Loop *L, std::map<Edge, float> &BackEdgeProbabilities);
  void calculateFrequencies(BasicBlock *BB, std::map<Edge, float> &BackEdgeProbabilities);
  void unmarkReachable(BasicBlock *BB, std::map<BasicBlock *, bool> &Visited);
  void propagateFrequencies(BasicBlock *BB, BasicBlock *Head, std::map<BasicBlock *, bool> &Visited, std::map<Edge, float> &BackEdgeProbabilities);
  void calculateBlockFrequency(BasicBlock *BB, std::map<BasicBlock *, bool> &Visited, std::map<Edge, float> &BackEdgeProbabilities);
  bool isBackEdge(Edge E);

  std::map<Edge, float> EdgeFrequencies;
  std::map<BasicBlock *, float> BlockFrequencies;
};

LocalFrequencies::LocalFrequencies() : FunctionPass(ID)
{
}

bool LocalFrequencies::runOnFunction(Function &F)
{
  std::map<Edge, float> BackEdgeProbabilities;
  SeansLLVM::BranchProbabilities &P = getAnalysis<SeansLLVM::BranchProbabilities>();

  // foreach edge do back_edge_probability(edge) = prob(edge)
  for (Function::iterator I = F.begin(), E = F.end(); I != E; ++I)
  {
    BasicBlock *BB = &*I;
    for (succ_iterator SI = succ_begin(BB), SE = succ_end(BB); SI != SE; ++SI)
    {
      BasicBlock *Successor = *SI;
      BackEdgeProbabilities[Edge(BB, Successor)] = P.getProb(BB, Successor);
    }
  }

  // foreach loop from inner-most to out-most do
  LoopInfo &LI = getAnalysis<LoopInfo>();
  for (LoopInfo::iterator I = LI.begin(), E = LI.end(); I != E; ++I)
  {
    processLoop(*I, BackEdgeProbabilities);
  }

  calculateFrequencies(&F.getEntryBlock(), BackEdgeProbabilities);

  return false;
}

void LocalFrequencies::processLoop(Loop *L, std::map<Edge, float> &BackEdgeProbabilities)
{
  // Process from inner-most to out-most
  for (Loop::iterator I = L->begin(), E = L->end(); I != E; ++I)
  {
    processLoop(*I, BackEdgeProbabilities);
  }

  calculateFrequencies(L->getHeader(), BackEdgeProbabilities);
}

void LocalFrequencies::calculateFrequencies(BasicBlock *BB, std::map<Edge, float> &BackEdgeProbabilities)
{
  std::map<BasicBlock *, bool> Visited;
  Function *F = BB->getParent();
  assert(F);
  for (Function::iterator I = F->begin(), E = F->end(); I != E; ++I)
  {
    Visited[&*I] = true;
  }
  unmarkReachable(BB, Visited);
  propagateFrequencies(BB, BB, Visited, BackEdgeProbabilities);
}

void LocalFrequencies::unmarkReachable(BasicBlock *BB, std::map<BasicBlock *, bool> &Visited)
{
  Visited[BB] = false;
  for (succ_iterator I = succ_begin(BB), E = succ_end(BB); I != E; ++I)
  {
    BasicBlock *Successor = *I;
    if (!Visited[Successor])
      continue;
    unmarkReachable(Successor, Visited);
  }
}

void LocalFrequencies::propagateFrequencies(BasicBlock *BB, BasicBlock *Head, std::map<BasicBlock *, bool> &Visited, std::map<Edge, float> &BackEdgeProbabilities)
{
  if (Visited[BB])
    return;

  // 1. find bfreq(b)
  if (BB == Head)
    BlockFrequencies[BB] = 1.0f;
  else
  {
    for (pred_iterator PI = pred_begin(BB), E = pred_end(BB); PI != E; ++PI)
    {
      BasicBlock *Predecessor = *PI;
      if (!Visited[Predecessor] && !isBackEdge(Edge(Predecessor, BB)))
      {
        DEBUG(dbgs() << "Why hasn't the pred been visited?\n");
        return;
      }
    }

    calculateBlockFrequency(BB, Visited, BackEdgeProbabilities);
  }

  // 2. calculate the frequencies of b's out edges
  Visited[BB] = true;

  SeansLLVM::BranchProbabilities &P = getAnalysis<SeansLLVM::BranchProbabilities>();
  for (succ_iterator I = succ_begin(BB), E = succ_end(BB); I != E; ++I)
  {
    BasicBlock *Successor = *I;
    Edge E(BB, Successor);
    EdgeFrequencies[E] = BlockFrequencies[BB] * P.getProb(BB, Successor);
    // update back_edge_prob(b->bi) so it can be used by outer loops to
    // calculate cyclic_probability of inner loops
    if (Successor == Head)
      BackEdgeProbabilities[E] = EdgeFrequencies[E];
  }

  // 3. propagate to successor blocks
  for (succ_iterator I = succ_begin(BB), E = succ_end(BB); I != E; ++I)
  {
    BasicBlock *Successor = *I;
    if (!isBackEdge(Edge(BB, Successor)))
      propagateFrequencies(Successor, Head, Visited, BackEdgeProbabilities);
  }
}

void LocalFrequencies::calculateBlockFrequency(BasicBlock *BB, std::map<BasicBlock *, bool> &Visited, std::map<Edge, float> &BackEdgeProbabilities)
{
  BlockFrequencies[BB] = 0;
  float CyclicProbability = 0;
  for (pred_iterator PI = pred_begin(BB), E = pred_end(BB); PI != E; ++PI)
  { 
    BasicBlock *Predecessor = *PI;
    if (isBackEdge(Edge(Predecessor, BB)))
    {
      CyclicProbability += BackEdgeProbabilities[Edge(Predecessor, BB)];
    }
    else
    {
      BlockFrequencies[BB] += EdgeFrequencies[Edge(Predecessor, BB)];
    }
  }

  if (CyclicProbability > (1.0f - EPSILON))
    CyclicProbability = 1.0f - EPSILON;

  BlockFrequencies[BB] /= 1.0f - CyclicProbability;
}

bool LocalFrequencies::isBackEdge(Edge E)
{
  LoopInfo &LI = getAnalysis<LoopInfo>();
  Loop *L = LI.getLoopFor(E.second);
  return L && (L->getHeader() == E.second) && L->contains(E.first);
}

} // namespace llvm
