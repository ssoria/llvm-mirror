//===- SeansBranchProbabilities.cpp ---------------------------------------===//
//
// This file implements algorithm 1 from "Static Brnch Frequency and Program
// Profile Analysis by Wu and Larus
//
//===----------------------------------------------------------------------===//

#ifndef SEAN_STATIC_PROF
#define SEAN_STATIC_PROF

#include "llvm/Instructions.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/PostDominators.h"
#include "llvm/Target/TargetInstrInfo.h"

using namespace llvm;

namespace SeansLLVM
{
  class BranchProbabilities : public FunctionPass
  {
  public:
    static char ID; // Pass identification, replacement for typeid

    BranchProbabilities();

    // We don't modify the program, so we preserve all analyses
    virtual void getAnalysisUsage(AnalysisUsage &AU) const;

    virtual bool runOnFunction(Function &F);

    float getProb(BasicBlock *A, BasicBlock *B);


  private:
    typedef DenseMap<BasicBlock *, float *> EdgeProb_lvl2;
    typedef DenseMap<BasicBlock *, EdgeProb_lvl2> EdgeProb;

    int CheckLoopBranchHeuristic();
    int CheckPointerHeuristic();
    int CheckIntegerHeuristic();
    int CheckFloatHeuristic();
    int CheckGuardHeuristic();
    bool CheckLoopHeaderHeuristic(int i);
    bool CheckReturnHeuristic(int i);
    template <class InstType> bool CheckInstHeuristic(int i);

    void PredictAsTaken(int i, float fProb, char *s);
    void PredictAsNotTaken(int i, float fProbH, char *s);


    LoopInfo *_LI;
    PostDominatorTree *_PDT;
    BasicBlock *_Succ[2];
    TerminatorInst *_TI;
    BasicBlock *_BB;
    float _fProb[2];
    bool _bPostDoms[2];
    Loop *_L;
    EdgeProb _EdgeProbs;

    // For block freq calculations
    bool BlockFreqPass(Function &F);
    void propagateFreq(BasicBlock *BB);
    void ProcessLoop(Loop *L);

    SmallPtrSet<BasicBlock *, 32> _Visited;
    EdgeProb _BackEdgeProb;
    EdgeProb _Freq;
    BasicBlock *_head;
    DenseMap<BasicBlock *, float *> _bfreq;

  }; // end of struct BranchProbabilities

}

#endif
