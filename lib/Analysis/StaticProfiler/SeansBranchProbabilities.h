//===- SeansBranchProbabilities.cpp ---------------------------------------===//
//
// This file implements algorithm 1 from "Static Branch Frequency and Program
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
#include "llvm/Support/raw_ostream.h"
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
    virtual void print(raw_ostream &O, const Module *M) const
    {
      O << myF->getName() << '\n';
      return;
      for (EdgeProb::const_iterator IA = _EdgeProbs.begin(), EA = _EdgeProbs.end(); IA != EA; ++IA)
      {
        BasicBlock *A = IA->first;
        EdgeProb_lvl2 Lvl2 = IA->second;
        for (EdgeProb_lvl2::const_iterator IB = Lvl2.begin(), EB = Lvl2.end(); IB != EB; ++IB)
        {
          BasicBlock *B = IB->first;
          float probability = *IB->second;

          O << "Function " << A->getParent()->getName() << ": Edge(" << A->getName() << ", " << B->getName() << ") = " << probability << '\n';
        }
      }
    }


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

    void PredictAsTaken(int i, float fProb, const char *s);
    void PredictAsNotTaken(int i, float fProbH, const char *s);


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
    Function *myF;

  }; // end of struct BranchProbabilities

}

#endif
