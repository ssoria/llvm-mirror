#include "llvm/Function.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Support/Debug.h"

#include "SeansBranchProbabilities.h"

namespace llvm {

#define EPSILON 0.01f

class LocalFrequencies : public FunctionPass
{
public:
  static char ID;
  LocalFrequencies();
  virtual bool runOnFunction(Function &F);
  float operator[](BasicBlock &BB) { return BlockFrequencies[&BB]; }

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

} // namespace llvm
