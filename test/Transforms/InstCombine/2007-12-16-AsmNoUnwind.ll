; RUN: opt %s -instcombine | llvm-dis | grep nounwind

define void @bar() {
entry:
        call void asm sideeffect "", "~{dirflag},~{fpsr},~{flags}"( )
        ret void
}
