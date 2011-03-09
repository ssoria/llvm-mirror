//===- SeansBranchProbabilities.cpp ---------------------------------------===//
//
// This file implements algorithm 1 from "Static Brnch Frequency and Program
// Profile Analysis by Wu and Larus
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "SeansBranchProbabilities"

#include "SeansLLVM/SeansBranchProbabilities.h"

#include "llvm/Constants.h" // ConstantInt
#include "llvm/DerivedTypes.h"
#include "llvm/Function.h" // Function
#include "llvm/Support/Debug.h" // DOUT
#include "llvm/Support/CFG.h" // pred_iterator

using namespace llvm;
using namespace SeansLLVM;

#define LOOP_BRANCH_TAKEN_PROB 0.88f
#define POINTER_TAKEN_PROB 0.60f
#define OPCODE_TAKEN_PROB 0.84f
#define GUARD_TAKEN_PROB 0.62f
#define LOOP_HEADER_TAKEN_PROB 0.75f
#define CALL_NOT_TAKEN_PROB 0.78f
#define STORE_NOT_TAKEN_PROB 0.55f
#define RETURN_NOT_TAKEN_PROB 0.72f

char BranchProbabilities::ID = 0;

RegisterPass<BranchProbabilities> Y("SeansBranchProbabilities",
"Algorithm 1 from Static Brnch Frequency and Program Profile Analysis");

BranchProbabilities::BranchProbabilities() : FunctionPass((intptr_t)&ID)
{
}

// We don't modify the program, so we preserve all analyses
void BranchProbabilities::getAnalysisUsage(AnalysisUsage &AU) const
{
  AU.setPreservesAll();
  AU.addRequired<LoopInfo>();
  AU.addRequired<PostDominatorTree>();
}

bool BranchProbabilities::runOnFunction(Function &F)
{
  DOUT << "Function: " << F.getName() << '\n';

  _LI = &getAnalysis<LoopInfo>();
  _PDT = &getAnalysis<PostDominatorTree>();

  // foreach block b with n successors and m back edge successors (m <= n)
  for (Function::iterator BBI = F.begin(), BBend = F.end(); BBI != BBend; ++BBI)
  {
    _BB = BBI;
    DOUT << " Block: " << _BB->getName();

    _TI = _BB->getTerminator();

    if (!_TI)
      continue;

    unsigned nSuccessors = _TI->getNumSuccessors();

    if (nSuccessors == 2)
    {
      _Succ[0] = _TI->getSuccessor(0);
      _Succ[1] = _TI->getSuccessor(1);
      DOUT << '(' << _Succ[0]->getName() << ", " << _Succ[1]->getName() << ')';

      _fProb[0] = 0.5f;
      _fProb[1] = 0.5f;

      _bPostDoms[0] = _PDT->dominates(_Succ[0], _BB);
      _bPostDoms[1] = _PDT->dominates(_Succ[1], _BB);

      _L = _LI->getLoopFor(_BB);

      int hasBranches = CheckLoopBranchHeuristic();
      if (hasBranches == 0)
      {
        _fProb[0] = 0.88f;
        _fProb[1] = 0.12f;
      }
      else if (hasBranches == 1)
      {
        _fProb[0] = 0.12f;
        _fProb[1] = 0.88f;
      }
      else
      {
        for (int i = 0; i < 2; i++)
        {
          if (CheckLoopHeaderHeuristic(i))
            PredictAsTaken(i, LOOP_HEADER_TAKEN_PROB, "LHH");
          if (CheckInstHeuristic<CallInst>(i))
            PredictAsNotTaken(i, CALL_NOT_TAKEN_PROB, "CH");
          if (CheckReturnHeuristic(i))
            PredictAsNotTaken(i, RETURN_NOT_TAKEN_PROB, "RH");
          if (CheckInstHeuristic<StoreInst>(i))
            PredictAsNotTaken(i, STORE_NOT_TAKEN_PROB, "SH");
        }

        PredictAsTaken(CheckPointerHeuristic(), POINTER_TAKEN_PROB, "PH");
        PredictAsTaken(CheckFloatHeuristic(), OPCODE_TAKEN_PROB, "OH");
        PredictAsTaken(CheckIntegerHeuristic(), OPCODE_TAKEN_PROB, "OH");
        PredictAsTaken(CheckGuardHeuristic(), GUARD_TAKEN_PROB, "GH");
      }

      _EdgeProbs[_BB][_Succ[0]] = new float(_fProb[0]);
      _EdgeProbs[_BB][_Succ[1]] = new float(_fProb[1]);
    }
    else if (nSuccessors > 0)
    {
      float *fProb = new float(1.0f / ((float)nSuccessors));

      for (succ_iterator BI = succ_begin(_BB), Bend = succ_end(_BB); BI != Bend;
           ++BI)
      {
        BasicBlock *BB = *BI;
        _EdgeProbs[_BB][BB] = fProb;
      }
    }
    DOUT << '\n';
  }

  DOUT << " STATS\n";
  for (Function::iterator BBI = F.begin(), BBend = F.end(); BBI != BBend; ++BBI)
  {
    _BB = BBI;
    DOUT << " Block: " << _BB->getName();

    for (succ_iterator BI = succ_begin(_BB), Bend = succ_end(_BB); BI != Bend;
         ++BI)
    {
      BasicBlock *BB = *BI;
      DOUT << " -> " << BB->getName() << "(" << getProb(_BB, BB) << ")";
    }
    DOUT << '\n';
  }
  DOUT << '\n';

  return BlockFreqPass(F);
}

bool BranchProbabilities::BlockFreqPass(Function &F)
{
  // init back_edge_prob to prob
  for (Function::iterator BBI = F.begin(), BBend = F.end(); BBI != BBend; ++BBI)
  {
    _BB = BBI;

    for (succ_iterator BI = succ_begin(_BB), Bend = succ_end(_BB); BI != Bend;
         ++BI)
    {
      BasicBlock *BB = *BI;
      _BackEdgeProb[_BB][BB] = new float(getProb(_BB, BB));
    }
  }

  // foreach loop from inner-most to out-most do
  for (LoopInfo::iterator LII = _LI->begin(), LIend = _LI->end(); LII != LIend;
       ++LII)
  {
    Loop *L = *LII;
    ProcessLoop(L);
  }

  DOUT << "processing entry to function\n";

  _head = &F.getEntryBlock();
  // mark all blocks reachable from head as not visited
  _Visited.clear();
  // and mark all other blocks as visited.
  // NOTE: Not necessary because they are not reachable!
  propagateFreq(_head);

  DOUT << " STATS\n";
  for (Function::iterator BBI = F.begin(), BBend = F.end(); BBI != BBend; ++BBI)
  {
    _BB = BBI;
    DOUT << " Block: " << _BB->getName() << ' ' << *_bfreq[_BB] << '\n';
  }
  DOUT << '\n';

  return false;
}

void BranchProbabilities::propagateFreq(BasicBlock *BB)
{
  // if BB has been visited then return
  if (_Visited.count(BB))
  {
    return;
  }

  // 1. find bfreq(BB)
  if (BB == _head)
  {
    _bfreq[BB] = new float(1.0f);
  }
  else
  {
    // in order to be a back edge, BB must be a loop header and
    // pred must be contained in the loop
    Loop *L = _LI->getLoopFor(BB);
    if (!(L && (L->getHeader() == BB)))
    {
      L = NULL;
    }

    for (pred_iterator BI = pred_begin(BB), Bend = pred_end(BB); BI != Bend;
         ++BI)
    {
      BasicBlock *BBp = *BI;
      if (!_Visited.count(BBp) && !(L && L->contains(BBp) &&
                                    (L->getLoopPreheader() != BBp)))
      {
        return;
       }
    }

    _bfreq[BB] = new float(0.0f);
    float cyclic_probability = 0.0f;
    for (pred_iterator BI = pred_begin(BB), Bend = pred_end(BB); BI != Bend;
         ++BI)
    {
      BasicBlock *BBp = *BI;
      if (L && L->contains(BBp) && (L->getLoopPreheader() != BBp))
        cyclic_probability += (*(_BackEdgeProb[BBp][BB]));
      else
        (*(_bfreq[BB])) += (*(_Freq[BBp][BB]));
    }
    if (cyclic_probability > 0.95f)
      cyclic_probability = 0.95f;
    (*(_bfreq[BB])) = (*(_bfreq[BB])) / (1.0f - cyclic_probability);
  }

  // 2. calculate the frequencies of b's out edges
  _Visited.insert(BB);
  for (succ_iterator BI = succ_begin(BB), Bend = succ_end(BB); BI != Bend;
       ++BI)
  {
    BasicBlock *BBs = *BI;
    float fFreq = getProb(BB, BBs) * (*(_bfreq[BB]));
    _Freq[BB][BBs] = new float(fFreq);

    // update back_edge_prob(BB, BBs) so it
    // can be used by outer loops to calculate
    // cyclic_probability of inner loops
    if (BBs == _head)
      (*(_BackEdgeProb[BB][BBs])) = fFreq;
  }

  // 3. propagate to successor blocks
  for (succ_iterator BI = succ_begin(BB), Bend = succ_end(BB); BI != Bend;
       ++BI)
  {
    BasicBlock *BBs = *BI;
    Loop *L = _LI->getLoopFor(BBs);
    if (!(L && (BBs == L->getHeader()) && L->contains(BB) &&
          (BB != L->getLoopPreheader())))
      propagateFreq(BBs);
  }
}

void BranchProbabilities::ProcessLoop(Loop *L)
{
  // inner-most first
  for (Loop::iterator LI = L->begin(), Lend = L->end(); LI != Lend; ++LI)
  {
    Loop *L = *LI;
    ProcessLoop(L);
  }

  DOUT << "processing loop ";
  L->print(DOUT);
  DOUT << '\n';
  _head = L->getHeader();
  // mark all blocks reachable from head as not visited
  _Visited.clear();
  // and mark all other blocks as visited.
  // NOTE: Not necessary because they are not reachable!
  propagateFreq(_head);
}

int BranchProbabilities::CheckLoopBranchHeuristic()
{
  for (Loop *L = _LI->getLoopFor(_BB); L; L = L->getParentLoop())
  {
    BasicBlock *loopHeader = L->getHeader();
    bool bLoops[2] = {false, false};
    bool bExits[2] = {false, false};

    for (int i = 0; i < 2; i++)
      if (loopHeader == _Succ[i])
        bLoops[i] = true;

    if (bLoops[0])
    {
      if (!bLoops[1])
        return 0;
      else
        continue;
    }
    else if (bLoops[1])
      return 1;

    for (int i = 0; i < 2; i++)
    {
      BasicBlock *BB = _Succ[i];

      if (!L->contains(BB))
        bExits[i] = true;
    }

    if (bExits[0] == bExits[1])
      continue;

    if (bExits[0])
      return 1;
    else
      return 0;
  }

  return -1;
}

int BranchProbabilities::CheckPointerHeuristic()
{
  // Heuristic fails if the last instruction is not a conditional branch
  BranchInst *BI = dyn_cast<BranchInst>(_TI);
  if ((!BI) || (BI->isUnconditional()))
    return -1;

  // All pointer comparisons are done with the icmp instruction
  ICmpInst *icmp = dyn_cast<ICmpInst>(BI->getCondition());
  if (!icmp)
    return -1;

  Value *v[2];
  v[0] = icmp->getOperand(0);
  v[1] = icmp->getOperand(1);

  // Make sure we're comparing pointers
  if (isa<PointerType>(v[0]->getType()))
  {
    assert(isa<PointerType>(v[1]->getType()) && "v[1] is not a pointer!");

    // Choose the prefered branch depending on if this is an eq or neq comp
    switch (icmp->getPredicate())
    {
    case ICmpInst::ICMP_EQ:
      return 1;
    case ICmpInst::ICMP_NE:
      return 0;
    default:
      assert("non-equality comparison of pointers");
      return -1;
    }
  }

  return -1;
}

int BranchProbabilities::CheckFloatHeuristic()
{
  // Heuristic fails if the last instruction is not a conditional branch
  BranchInst *BI = dyn_cast<BranchInst>(_TI);
  if ((!BI) || (BI->isUnconditional()))
    return -1;

  // All float comparisons are done with the fcmp instruction
  FCmpInst *fcmp = dyn_cast<FCmpInst>(BI->getCondition());
  if (!fcmp)
    return -1;

  // Choose the prefered branch depending on if this is an eq or neq comp
  switch (fcmp->getPredicate())
  {
  case FCmpInst::FCMP_OEQ:
  case FCmpInst::FCMP_UEQ:
    return 1;
  case FCmpInst::FCMP_ONE:
  case FCmpInst::FCMP_UNE:
    return 0;
  case FCmpInst::FCMP_FALSE:
  case FCmpInst::FCMP_TRUE:
    assert("true or false predicate should have been folded!");
  default:
    return -1;
  }
}

int BranchProbabilities::CheckIntegerHeuristic()
{
  // Heuristic fails if the last instruction is not a conditional branch
  BranchInst *BI = dyn_cast<BranchInst>(_TI);
  if ((!BI) || (BI->isUnconditional()))
    return -1;

  // All integer comparisons are done with the icmp instruction
  ICmpInst *icmp = dyn_cast<ICmpInst>(BI->getCondition());
  if (!icmp)
    return -1;

  Value *v[2];
  v[0] = icmp->getOperand(0);
  v[1] = icmp->getOperand(1);

  // If neither is a constant, nothing to do
  if (!isa<ConstantInt>(v[0]) && !isa<ConstantInt>(v[1]))
    return -1;

  // If we're dealing with something other than ints, nothing to do
  if (!isa<IntegerType>(v[0]->getType()))
    return -1;

  // Get comparison
  ICmpInst::Predicate pred = icmp->getPredicate();

  // Eq and Not Eq are easy cases
  if (pred == ICmpInst::ICMP_EQ)
    return 1;
  else if (pred == ICmpInst::ICMP_NE)
    return 0;

  ConstantInt *CI = dyn_cast<ConstantInt>(v[1]);
  // If the right side isn't a constant, swap the predicate so we can pretend
  if (!CI)
  {
    pred = icmp->getSwappedPredicate();
    CI = cast<ConstantInt>(v[0]);
  }

  // Choose the appropriate branch depending on the const val and predicate
  if (CI->isZero())
  {
    switch (pred)
    {
    case ICmpInst::ICMP_UGE:
      assert("UGE zero always returns true");
      return -1;
    case ICmpInst::ICMP_ULT:
      assert("ULT zero always returns false");
      return -1;
    case ICmpInst::ICMP_UGT:
    case ICmpInst::ICMP_SGT:
    case ICmpInst::ICMP_SGE:
      return 0;
    case ICmpInst::ICMP_ULE:
    case ICmpInst::ICMP_SLT:
    case ICmpInst::ICMP_SLE:
      return 1;
    default:
      return -1;
    }
  } 
  else if (CI->isOne())
  {
    switch (pred)
    {
    case ICmpInst::ICMP_UGE:
    case ICmpInst::ICMP_SGE:
      return 0;
    case ICmpInst::ICMP_ULT:
    case ICmpInst::ICMP_SLT:
      return 1;
    default:
      return -1;
    }
  }
  else if (CI->isAllOnesValue())
  {
    switch (pred)
    {
    case ICmpInst::ICMP_SGT:
      return 0;
    case ICmpInst::ICMP_SLE:
      return 1;
    default:
      return -1;
    }
  }

  return -1;
}

// Predict that a comparison in which a register is an operand, the register is
// used before being defined in a successor block, and the successor block
// does not post-dominate will reach the successor block.
int BranchProbabilities::CheckGuardHeuristic()
{
  BranchInst *BI = dyn_cast<BranchInst>(_TI);
  bool bUses[2] = {false, false};

  // If we don't have a conditional branch, abandon
  if ((!BI) || (BI->isUnconditional()))
    return -1;

  // If the condition is not immediately dependent on a comparison, abandon
  CmpInst *cmp = dyn_cast<CmpInst>(BI->getCondition());
  if (!cmp)
    return -1;

  for (int i = 0; i < 2; i++)
  {
    if (_bPostDoms[i])
      continue;

    // Get the values being compared
    Value *v = cmp->getOperand(i);

    // For all uses of the first value check if the use post-dominates
    for (Value::use_iterator UI = v->use_begin(), UE = v->use_end();
         UI != UE; ++UI)
    {
      // if the use is not an instruction, skip it
      Instruction *I = dyn_cast<Instruction>(*UI);
      if (!I)
        continue;

      BasicBlock *UsingBlock = I->getParent();

      // Check if the use is in either successor
      for (int i = 0; i < 2; i++)
        if (UsingBlock == _Succ[i])
          bUses[i] = true;
    }
  }

  if (bUses[0] == bUses[1])
    return -1;
  if (bUses[0])
    return 0;
  else
    return 1;
}

bool BranchProbabilities::CheckLoopHeaderHeuristic(int i)
{
  if (_bPostDoms[i])
    return false;

  BasicBlock *BB = _Succ[i];

  // Get the loop, if any
  Loop *L = _LI->getLoopFor(BB);

  // If it's either a loop header or preheader, return true
  if (L && (L != _L))
    return true;

  // Did not find a loop header or preheader
  return false;
}

template <class InstType>
bool BranchProbabilities::CheckInstHeuristic(int i)
{
  if (_bPostDoms[i])
    return false;

  BasicBlock *BB = _Succ[i];

  // Loop through all instructions in the block
  for (BasicBlock::iterator I = BB->begin(), IEnd = BB->end(); I != IEnd; ++I)
  {
    // Return true if we have the specified instruction
    if (isa<InstType>(I))
      return true;
  }

  // Did not find the instruction
  return false;
}

bool BranchProbabilities::CheckReturnHeuristic(int i)
{
  BasicBlock *BB = _Succ[i];

  // Loop through all instructions in the block
  for (BasicBlock::iterator I = BB->begin(), IEnd = BB->end(); I != IEnd; ++I)
  { 
      // Return true if we have the specified instruction
    if (isa<ReturnInst>(I))
      return true;
  }

  return false;
}

void BranchProbabilities::PredictAsNotTaken(int i, float fProb, char *s)
{
  int iOther = (i * -1) + 1;
  PredictAsTaken(iOther, fProb, s);
}

void BranchProbabilities::PredictAsTaken(int i, float fProbH, char *s)
{
  if ((i != 0) && (i != 1))
    return;

  DOUT << " " << s << i;

  int iOther = (i * -1) + 1;
  float fProbNotH = 1.0f - fProbH;
  float fDivisor = (_fProb[i] * fProbH) + (_fProb[iOther] * fProbNotH);
  _fProb[i] = _fProb[i] * fProbH / fDivisor;
  _fProb[iOther] = _fProb[iOther] * fProbNotH / fDivisor;
}

float BranchProbabilities::getProb(BasicBlock *A, BasicBlock *B)
{
  return (*(_EdgeProbs[A][B]));
}

