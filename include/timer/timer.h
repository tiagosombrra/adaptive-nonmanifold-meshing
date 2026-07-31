#ifndef TIMER_TIMER_H
#define TIMER_TIMER_H

#include <stdio.h>

#include <cstdio>
#include <fstream>
#include <sstream>

#include "../data/definitions.h"

using namespace std;
extern double TIME_READ_FILE;
extern std::string NAME_MODEL;
extern std::string NUMBER_PROCESS;

//[0]= Inicialização
//[1]= Estimativa de carga
//[2]= Geração da Malha Inicial
//[3]= Adaptação das curvas
//[4]= Adaptação do domínio
//[5]= ...
//[6]= MediaGauss
//[7]= Calculo do erro
//[8]= Overhead
//[9]= Timer send and recv process
//[10]= Full without file I/O
//[11]= Save output

class Timer {
 public:
  Timer();
  Timer(int size_rank, int size_thread, int size_type);
  bool OpenFile(string);
  void CloseFile();
  bool DeleteFile(string);

  void InitTimerParallel(int rank, int thread, int type);
  void EndTimerParallel(int rank, int thread, int type);
  void CalculateTime(int rank, int thread, int type);
  void PrintTime();
  void PrintTime(int rank_process);
  double GetRankThreadTime(int rank, int thread, int type);
  vector<double> GetMaxTime();
  vector<double> GetMinTime();

  vector<vector<vector<timeval>>> GetTimerParallelInit() const;
  void SetTimerParallelInit(const vector<vector<vector<timeval>>> &value);

  vector<vector<vector<timeval>>> GetTimerParallelEnd() const;
  void SetTimerParallelEnd(const vector<vector<vector<timeval>>> &value);

  vector<vector<vector<double>>> GetTimerParallel() const;
  void SetTimerParallel(const vector<vector<vector<double>>> &value);

  vector<vector<vector<double>>> timer_parallel_init_mpi_;
  vector<vector<vector<double>>> timer_parallel_end_mpi_;
  vector<vector<vector<timeval>>> timer_parallel_init_;
  vector<vector<vector<timeval>>> timer_parallel_end_;
  vector<vector<vector<double>>> timer_parallel_;

  string location_name_;
  fstream file_;
};

#endif  // TIMER_TIMER_H
