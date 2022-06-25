#pragma once

//Repetitive macro power to improve timing critical power speed https://stackoverflow.com/a/8556436
//Provide macro expansion of pow up to power of 10

#define REP1(X) X
#define REP2(X) REP1(X) * X 
#define REP3(X) REP2(X) * X 
#define REP4(X) REP3(X) * X 
#define REP5(X) REP4(X) * X 
#define REP6(X) REP5(X) * X 
#define REP7(X) REP6(X) * X 
#define REP8(X) REP7(X) * X 
#define REP9(X) REP8(X) * X 
#define REP10(X) REP9(X) * X

#define CKPOW(X, ONES) REP##ONES(X)