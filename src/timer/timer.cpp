#include "../../include/timer/timer.h"

extern double MAX_TIME;

Timer::Timer() {
  this->timer_parallel_init_.resize(1);
  this->timer_parallel_end_.resize(1);
  this->timer_parallel_init_mpi_.resize(1);
  this->timer_parallel_end_mpi_.resize(1);
  this->timer_parallel_.resize(1);

  this->timer_parallel_init_[0].resize(1);
  this->timer_parallel_end_[0].resize(1);
  this->timer_parallel_init_mpi_[0].resize(1);
  this->timer_parallel_end_mpi_[0].resize(1);
  this->timer_parallel_[0].resize(1);

  this->timer_parallel_init_[0][0].resize(12);
  this->timer_parallel_end_[0][0].resize(12);
  this->timer_parallel_init_mpi_[0][0].resize(12);
  this->timer_parallel_end_mpi_[0][0].resize(12);
  this->timer_parallel_[0][0].resize(12);

  for (int i = 0; i < 12; i++) {
    this->timer_parallel_[0][0][i] = 0.0;
  }
}

Timer::Timer(int size_rank, int size_thread, int size_type) {
  size_rank = (size_rank == 0) ? 1 : size_rank;
  size_thread = (size_thread == 0) ? 1 : size_thread;

  this->timer_parallel_init_.resize(size_rank);
  this->timer_parallel_end_.resize(size_rank);
  this->timer_parallel_init_mpi_.resize(size_rank);
  this->timer_parallel_end_mpi_.resize(size_rank);
  this->timer_parallel_.resize(size_rank);

  for (int i = 0; i < size_rank; i++) {
    this->timer_parallel_init_[i].resize(size_thread);
    this->timer_parallel_end_[i].resize(size_thread);
    this->timer_parallel_init_mpi_[i].resize(size_thread);
    this->timer_parallel_end_mpi_[i].resize(size_thread);
    this->timer_parallel_[i].resize(size_thread);

    for (int j = 0; j < size_thread; j++) {
      this->timer_parallel_init_[i][j].resize(size_type);
      this->timer_parallel_end_[i][j].resize(size_type);
      this->timer_parallel_init_mpi_[i][j].resize(size_type);
      this->timer_parallel_end_mpi_[i][j].resize(size_type);
      this->timer_parallel_[i][j].resize(size_type);

      for (int l = 0; l < size_type; l++) {
        this->timer_parallel_[i][j][l] = 0.0;
      }
    }
  }
}

bool Timer::OpenFile(string location_name) {
  // Metodo de criação(caso não exista) ou abertura de arquivo(caso exista).

  // localização e nome do arquivo
  this->location_name_ = location_name;
  file_.open(location_name.c_str(), fstream::out | fstream::ate);
  file_ << "Arquivo gerado pelo apMesh para medir o tempo detalaha de execução "
           "do programa"
        << endl
        << endl;

  return file_.is_open();
}

void Timer::CloseFile() { file_.close(); }

bool Timer::DeleteFile(string file_name) {
  if (std::remove(file_name.c_str()) != 0) {
    return false;
  } else {
    return true;
  }
}

void Timer::InitTimerParallel(int rank, int thread, int type) {
  timeval begin;
  gettimeofday(&begin, 0);
  this->timer_parallel_init_[rank][thread][type] = begin;
}

void Timer::EndTimerParallel(int rank, int thread, int type) {
  timeval end;
  gettimeofday(&end, 0);
  this->timer_parallel_end_[rank][thread][type] = end;
  CalculateTime(rank, thread, type);
}

void Timer::CalculateTime(int rank, int thread, int type) {
  long seconds = (this->timer_parallel_end_[rank][thread][type]).tv_sec -
                 (this->timer_parallel_init_[rank][thread][type]).tv_sec;
  long microseconds = (this->timer_parallel_end_[rank][thread][type]).tv_usec -
                      (this->timer_parallel_init_[rank][thread][type]).tv_usec;
  this->timer_parallel_[rank][thread][type] +=
      static_cast<double>(seconds) +
      static_cast<double>(microseconds) * 1e-6;
}

void Timer::PrintTime() {
  vector<double> max = GetMaxTime();
  cout << "Send: " << max[0] << endl;           // Send
  cout << "Estimate: " << max[1] << endl;       // Estimativa de carga
  cout << "Init mesh: " << max[2] << endl;      // Geração da malha inicial
  cout << "Adapt curve: " << max[3] << endl;    // Adaptação das curvas
  cout << "Adapt domain: " << max[4] << endl;   // Adaptação do domínio
  cout << "Read file: " << max[5] << endl;      // Leitura arquivo
  cout << "Calcule gauss: " << max[6] << endl;  // MediaGauss
  cout << "Calcule error: " << max[7] << endl;  // Calculo do erro
  cout << "Overhead: "
       << max[10] - max[11] - max[9] - max[7] - max[6] - max[5] - max[4] - max[3] -
              max[2] - max[1] - max[0]
       << endl;                                  // Overhead
  cout << "Recv: " << max[9] << endl;            // Recv
  cout << "Save output: " << max[11] << endl;    // Save output
  cout << "Full: " << max[10] - max[5] - max[11] << endl;  // Full without file I/O
}

void Timer::PrintTime(int rank_process) {
  if (rank_process == 0) {
    vector<double> max = GetMaxTime();
    cout << "Send: " << max[0] << endl;           // Send
    cout << "Estimate: " << max[1] << endl;       // Estimativa de carga
    cout << "Init mesh: " << max[2] << endl;      // Geração da malha inicial
    cout << "Adapt curve: " << max[3] << endl;    // Adaptação das curvas
    cout << "Adapt domain: " << max[4] << endl;   // Adaptação do domínio
    cout << "Read file: " << max[5] << endl;      // Leitura arquivo
    cout << "Calcule gauss: " << max[6] << endl;  // MediaGauss
    cout << "Calcule error: " << max[7] << endl;  // Calculo do erro
    cout << "Overhead: "
         << max[10] - max[11] - max[9] - max[7] - max[6] - max[5] - max[4] - max[3] -
                max[2] - max[1] - max[0]
         << endl;                                  // Overhead
    cout << "Recv: " << max[9] << endl;            // Recv
    cout << "Save output: " << max[11] << endl;    // Save output
    cout << "Full: " << max[10] - max[5] - max[11] << endl;  // Full without file I/O

  } else {
    vector<double> max = GetMaxTime();
    cout << "Send: " << max[0] << endl;           // Send
    cout << "Estimate: " << max[1] << endl;       // Estimativa de carga
    cout << "Init mesh: " << max[2] << endl;      // Geração da malha inicial
    cout << "Adapt curve: " << max[3] << endl;    // Adaptação das curvas
    cout << "Adapt domain: " << max[4] << endl;   // Adaptação do domínio
    cout << "Read file: " << max[5] << endl;      // Leitura arquivo
    cout << "Calcule gauss: " << max[6] << endl;  // MediaGauss
    cout << "Calcule error: " << max[7] << endl;  // Calculo do erro
    cout << "Overhead: "
         << max[10] - max[11] - max[9] - max[7] - max[6] - max[5] - max[4] - max[3] -
                max[2] - max[1] - max[0]
         << endl;                                  // Overhead
    cout << "Recv: " << max[9] << endl;            // Recv
    cout << "Save output: " << max[11] << endl;    // Save output
    cout << "Full: " << max[10] - max[5] - max[11] << endl;  // Full without file I/O
  }
}

double Timer::GetRankThreadTime(int rank, int thread, int type) {
  return timer_parallel_[rank][thread][type];
}

vector<double> Timer::GetMaxTime() {
  vector<double> max;
  max.resize(this->timer_parallel_[0][0].size());

  for (unsigned int i = 0; i < this->timer_parallel_[0][0].size(); ++i) {
    max[i] = 0;
  }

  for (unsigned int i = 0; i < this->timer_parallel_.size(); ++i) {
    for (unsigned int j = 0; j < timer_parallel_[i].size(); ++j) {
      for (unsigned int l = 0; l < this->timer_parallel_[i][j].size(); ++l) {
        if (this->timer_parallel_[i][j][l] > max[l]) {
          max[l] = this->timer_parallel_[i][j][l];
        }
      }
    }
  }

  return max;
}

vector<double> Timer::GetMinTime() {
  vector<double> min;
  min.resize(this->timer_parallel_[0][0].size());

  for (unsigned int i = 0; i < this->timer_parallel_[0][0].size(); ++i) {
    min[i] = MAX_TIME;
  }

  for (unsigned int i = 0; i < this->timer_parallel_.size(); ++i) {
    for (unsigned int j = 0; j < timer_parallel_[i].size(); ++j) {
      for (unsigned int l = 0; l < this->timer_parallel_[i][j].size(); ++l) {
        if (this->timer_parallel_[i][j][l] < min[l]) {
          min[l] = this->timer_parallel_[i][j][l];
        }
      }
    }
  }

  return min;
}

vector<vector<vector<timeval> > > Timer::GetTimerParallelInit() const {
  return timer_parallel_init_;
}

void Timer::SetTimerParallelInit(
    const vector<vector<vector<timeval> > > &value) {
  timer_parallel_init_ = value;
}

vector<vector<vector<timeval> > > Timer::GetTimerParallelEnd() const {
  return timer_parallel_end_;
}

void Timer::SetTimerParallelEnd(
    const vector<vector<vector<timeval> > > &value) {
  timer_parallel_end_ = value;
}

vector<vector<vector<double> > > Timer::GetTimerParallel() const {
  return timer_parallel_;
}

void Timer::SetTimerParallel(const vector<vector<vector<double> > > &value) {
  timer_parallel_ = value;
}
