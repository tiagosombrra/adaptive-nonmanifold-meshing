#include "../../../include/data/curve/curve_adaptive_parametric.h"

extern double DELTA;
extern double TOLERANCE;

// Global function
vector<CurveAdaptiveParametric*> ptr_aux;

bool Compare(PointAdaptive* point1, PointAdaptive* point2) {
  double parameter1, parameter2;

#if USE_OPENMP
  parameter1 = ptr_aux[omp_get_thread_num()]->FindParameterByPoint(*point1);
  parameter2 = ptr_aux[omp_get_thread_num()]->FindParameterByPoint(*point2);
#else
  parameter1 = ptr_aux[0]->FindParameterByPoint(*point1);
  parameter2 = ptr_aux[0]->FindParameterByPoint(*point2);
#endif  // USE_OPENMP

  return parameter1 < parameter2;
}

CurveAdaptiveParametric::CurveAdaptiveParametric() {
  mat_geo_gx_.setZero(4, 1);
  mat_geo_gy_.setZero(4, 1);
  mat_geo_gz_.setZero(4, 1);
  mat_base_.setZero(4, 4);
  mat_parameters_.setZero(1, 4);
}

CurveAdaptiveParametric::CurveAdaptiveParametric(const PointAdaptive point0,
                                                 const PointAdaptive point1)
    : point0_(point0), point1_(point1) {
  mat_geo_gx_.setZero(4, 1);
  mat_geo_gy_.setZero(4, 1);
  mat_geo_gz_.setZero(4, 1);
  mat_base_.setZero(4, 4);
  mat_parameters_.setZero(1, 4);
}

CurveAdaptiveParametric::CurveAdaptiveParametric(
    CurveAdaptiveParametric* curve_adaptive_parametric)
    : CurveAdaptive(curve_adaptive_parametric) {
  this->point0_ = curve_adaptive_parametric->point0_;
  this->point1_ = curve_adaptive_parametric->point1_;
  this->mat_geo_gx_ = curve_adaptive_parametric->mat_geo_gx_;
  this->mat_geo_gy_ = curve_adaptive_parametric->mat_geo_gy_;
  this->mat_geo_gz_ = curve_adaptive_parametric->mat_geo_gz_;
  this->mat_base_ = curve_adaptive_parametric->mat_base_;
  this->mat_parameters_ = curve_adaptive_parametric->mat_parameters_;
}

CurveAdaptiveParametric::~CurveAdaptiveParametric() {
  // delete &mat_geo_gx_;
  // delete &mat_geo_gy_;
  // delete &mat_geo_gz_;
  // delete &mat_base_;
  // delete &mat_parameters_;
}

// calcula o comprimento de curva de p1 a p2
double CurveAdaptiveParametric::CalculateLengthPoints(
    const PointAdaptive& point1, const PointAdaptive& point2) {
  // parâmetro do ponto p1
  double parameter1 = FindParameterByPoint(point1);
  // parâmetro do ponto p2
  double parameter2 = FindParameterByPoint(point2);

  if (parameter1 < parameter2) {
    return CalculateParametricLength(parameter1, parameter2);
  }
  if (parameter1 > parameter2) {
    return CalculateParametricLength(parameter2, parameter1);
  }

  return 0;
}

// calcula o comprimento de curva de point_0_ até p
double CurveAdaptiveParametric::CalculateLengthPoint(
    const PointAdaptive& point) {
  return CalculateParametricLength(0, FindParameterByPoint(point));
}

// calcula comprimento total "L" da curva
void CurveAdaptiveParametric::CalculateLengthCurve() {
  this->length_ = CalculateParametricLength(0, 1);
}

double CurveAdaptiveParametric::CalculateCurvature(double) { return -1; }

// calcula o comprimento de curva entre os parâmetros t1 e t2
double CurveAdaptiveParametric::CalculateParametricLength(double parameter1,
                                                          double parameter2) {
  /* A cada incremento de t,
     p recebe as coordenadas do ponto na curva correspondente
     a t; a distância entre p e o ponto anterior
     p_ant é calculada.

     Essa distância é somada a distância total d. */

  /*if ( t1 > t2 ) // caso t1 esteja à direita de t2 na curva
  {
      double aux = t1;
      t1 = t2;
      t2 = aux;
  }

  Ponto p; // ponto da iteração atual
  Ponto p_ant; // ponto da iteracao anterior

  double t = t1; // parâmetro de iteração
  double d = 0.0; // distância total entre os dois pontos

  // 1. cria Ponto 'p_ant'
  p_ant = this->Parameterize ( t1 );

  do
  {
      t += DELTA;

      // 2. cria Ponto 'p'
      p = this->Parameterize ( t );

      // incrementa distancia
      d += p_ant.distanciaPara ( p );

      // copia o conteudo de 'p' para o ponto anterior
      p_ant = p;
  }
  while ( t <= ( t2 - DELTA ) );

  return d;*/

  // encontrando o comprimento por gauss legendre
  return CalculateLength(parameter1, parameter2, 9, 4);
}

// encontra o parâmetro t de um dado ponto p na curva
double CurveAdaptiveParametric::FindParameterByPoint(
    const PointAdaptive& point) {
  struct DistanceFunction : public Data::Numerical::EquationRootFunction {
    const PointAdaptive* point;
    CurveAdaptiveParametric* curva;

    double min() { return 0.0; };
    double max() { return 1.0; };

    using Data::Numerical::EquationRootFunction::f;
    double f(double parameter) {
      PointAdaptive point1 = curva->FindPointByParameter(parameter);
      return point->CalculateDistance(point1);
    };
  };

  DistanceFunction distance_function;
  distance_function.curva = this;
  distance_function.point = &point;

  Data::Numerical::ClosestBisectionEquationRoot closet_bisection_eq_root;

  bool return_function = true;

  double parameter =
      closet_bisection_eq_root.execute(&distance_function, return_function);

  return return_function ? parameter : -1.0;

  //    // end markos

  //        long double d_min = 1.0e50; // distância mínima entre p e a curva
  //        long double di = 0; // distância do palpite até p
  //        long double t_min = 0.0; // parâmetro do pondo da curva mais próximo
  //        a p Ponto *pi = new Ponto; // ponto de palpite

  //        for ( long double t = 0.0; t <= 1.0; t += DELTA )
  //        {
  //            *pi = Parameterize ( t );
  //            di = pi->distanciaPara ( p );
  //            if ( di < d_min )
  //            {
  //                    d_min = di;
  //                    t_min = t;
  //            }
  //        }

  //        delete pi;

  //        return t_min;

  //	/* Método utilizando projeção vetorial, de forma
  //		semelhante ao método da bisseção. Quanto mais próximo
  //		p estiver dos extremos, menor será a precisão */

  //	double frp = DELTA; // fator de reposicionamento
  //	double tm = 0.5; // método da bisseção
  //	double delta_t = 0.0; // o quanto o parâmetro terá de percorrer
  //	double d_point_0_ = this->point_0_.distanciaPara ( p ); // distância de
  // point_0_ a p 	double d_P1 = this->point_1_.distanciaPara ( p ); //
  // distância de point_1_ a p

  //	// o ponto está muuuuito próximo de point_0_ ?
  //	if ( d_point_0_ <= DELTA )
  //		tm = 0.0; // retorna tm = 0.0

  //	// o ponto está muuuuito próximo de point_1_ ?
  //	if ( d_P1 <= DELTA )
  //		tm = 1.0; // retorna tm = 1.0

  //	Ponto Si;

  //	do
  //	{
  //		// 1. cria Si (modificou T)
  //		Si = this->Parameterize ( tm );

  //		// 2. cria Vj
  //		Vetor Vj ( Si, p );

  //		// 3. cria Tu (modificou T)
  //		Vetor Tu ( this->Qt ( tm ) );

  //		// 4. calcula a projecao de Vj em Tu
  //		delta_t = Vj ^ Tu;

  //		// 5. calcula 'tm' (este eh o ultimo valor de 'tm' calculado)
  //		tm += delta_t * frp;
  //	}
  //	while ( Si.distanciaPara ( p ) > TOLERANCE );

  //	if ( tm < DELTA ) tm = 0.0; // t está muito próximo a 0
  //	else if ( tm > ( 1.0 - DELTA ) ) tm = 1.0; // t está muito próximo a 1

  //	return tm; // retorna o ultimo valor de 'tm' calculado
}

// encontra as coordenadas 3D de um ponto p de parâmetro t
PointAdaptive CurveAdaptiveParametric::FindPointByParameter(double t) {
  // Parameterize:
  //
  //  -> ALTERA a matriz T
  //  -> usa calculaPonto_t
  //

  this->mat_parameters_(0, 0) = t * t * t;
  this->mat_parameters_(0, 1) = t * t;
  this->mat_parameters_(0, 2) = t;
  this->mat_parameters_(0, 3) = 1;

  return CalculatePointT();
}

// calcula o vetor tangente no parâmetro t
VectorAdaptive CurveAdaptiveParametric::CalculateGradientByParameter(double t) {
  // Qt:
  //
  //  -> ALOCA um Ponto (mas destroi aqui mesmo)
  //	 -> ALOCA um Vetor (retorna ele)
  //  -> ALTERA a matriz T !!!
  //

  PointAdaptive point;

  this->mat_parameters_(0, 0) = 3 * t * t;
  this->mat_parameters_(0, 1) = 2 * t;
  this->mat_parameters_(0, 2) = 1;
  this->mat_parameters_(0, 3) = 0;

  point = CalculatePointT();
  VectorAdaptive vector(point);

  return vector;
}

// retorna o ponto que fica na metade de um segmento
PointAdaptive CurveAdaptiveParametric::CalculateMidpointByPoints(
    const PointAdaptive& point1, const PointAdaptive& point2) {
  return this->FindPointByParameter(this->CalculateMidparameterByParamters(
      this->FindParameterByPoint(point1), this->FindParameterByPoint(point2)));
}

double CurveAdaptiveParametric::CalculateMidparameterByParamters(
    double parameter1, double parameter2) {
  if (true) {
    return CalculateMidpointBisection(parameter1, parameter2);
  }
  // tamanho total do segmento
  double length_total;
  // metade do comprimento do segmento
  double length_half;
  // comprimento da curva até palpite
  double length_hint_total;
  // parâmetro do palpite inicial
  double length_hint_half;

  // 1. Calcule o comprimento do segmento
  length_total = this->CalculateParametricLength(parameter1, parameter2);
  length_half = length_total / 2.0;

  // 2. faz um palpite inicial
  //
  length_hint_half = (parameter1 + parameter2) / 2.0;

  // 3. p1 é menor que p2 ?
  //
  // p1 é "menor" que p2
  if (parameter1 < length_hint_half) {
    length_hint_total = CalculateParametricLength(parameter1, length_hint_half);
    // p1 é "maior" que p2
  } else if (parameter1 > length_hint_half) {
    length_hint_total = CalculateParametricLength(parameter2, length_hint_half);
    // p1 e p2 são muito próximos ou iguais
  } else {
    return parameter1;
  }

  // 4. length_hint_total é maior que L/2 ?
  //
  //    -> Achar um parametro que dá o ponto aproximado
  //       na metade da curva
  //
  if (length_hint_total > length_half) {
    while (length_hint_total >= length_half) {
      length_hint_total -=
          CalculateParametricLength(length_hint_half - DELTA, length_hint_half);
      length_hint_half -= DELTA;
    }
    length_hint_half += DELTA;
  } else {
    while (length_hint_total <= length_half) {
      length_hint_total +=
          CalculateParametricLength(length_hint_half, length_hint_half + DELTA);
      length_hint_half += DELTA;
    }
    length_hint_half -= DELTA;
  }

  return length_hint_half;
}

// ordena a lista de pontos de acordo com suas coordenadas paramétricas
void CurveAdaptiveParametric::SortPointsByParameters() {
#if USE_OPENMP
  ptr_aux[omp_get_thread_num()] = this;
#else
  ptr_aux[0] = this;
#endif  // USE_OPENMP

  this->points_.sort(Compare);
}

void CurveAdaptiveParametric::UpdateParameters(
    const list<double> new_parameters) {
  this->parameters_ = new_parameters;
}

Matrix4x1 CurveAdaptiveParametric::GetMatGeoGx() const {
  return this->mat_geo_gx_;
}

void CurveAdaptiveParametric::SetMatGeoGx(const Matrix4x1& mat_geo_gx) {
  this->mat_geo_gx_ = mat_geo_gx;
}

Matrix4x1 CurveAdaptiveParametric::GetMatGeoGy() const {
  return this->mat_geo_gy_;
}

void CurveAdaptiveParametric::SetMatGeoGy(const Matrix4x1& mat_geo_gy) {
  this->mat_geo_gy_ = mat_geo_gy;
}

Matrix4x1 CurveAdaptiveParametric::GetMatGeoGz() const {
  return this->mat_geo_gz_;
}

void CurveAdaptiveParametric::SetMatGeoGz(const Matrix4x1& mat_geo_gz) {
  this->mat_geo_gz_ = mat_geo_gz;
}

Matrix4x4 CurveAdaptiveParametric::GetMatBase() const {
  return this->mat_base_;
}

void CurveAdaptiveParametric::SetMatBase(Matrix4x4& mat_base) {
  this->mat_base_ = mat_base;
}

Matrix1x4 CurveAdaptiveParametric::GetMatParameters() const {
  return this->mat_parameters_;
}

void CurveAdaptiveParametric::SetMatParameters(
    const Matrix1x4& mat_parameters) {
  this->mat_parameters_ = mat_parameters;
}

PointAdaptive CurveAdaptiveParametric::GetPoint0() const {
  return this->point0_;
}

void CurveAdaptiveParametric::SetPoint0(const PointAdaptive& point0) {
  this->point0_ = point0;
}

PointAdaptive CurveAdaptiveParametric::GetPoint1() const {
  return this->point1_;
}

void CurveAdaptiveParametric::SetPoint1(const PointAdaptive& point1) {
  this->point1_ = point1;
  CalculateLengthCurve();
}

void CurveAdaptiveParametric::SetPoint0Point1(const PointAdaptive& point0,
                                              const PointAdaptive& point1) {
  this->point0_ = point0;
  this->point1_ = point1;
}

// faz as multiplicações necessárias para 'parametriar ( t )' e 'Qt ( t )'
PointAdaptive CurveAdaptiveParametric::CalculatePointT() {
  PointAdaptive point;

  // C = ( T * ( M * G ) )
  point.SetX((this->GetMatParameters() * (this->GetMatGeoGx()))(0, 0));
  point.SetY((this->GetMatParameters() * (this->GetMatGeoGy()))(0, 0));
  point.SetZ((this->GetMatParameters() * (this->GetMatGeoGz()))(0, 0));

  return point;
}

double CurveAdaptiveParametric::CalculateLength(double t1, double t2, int parts,
                                                int points) {
  double total = 0.0, ui = 0.0, uf = t1;
  double partitionLength = (t2 - t1) / (double)parts;

  for (int k = 1; k <= parts; k++) {
    double localTotal = 0.0;

    ui = uf;
    uf = t1 + (double)k * partitionLength;

    for (int i = 0; i < points; i++) {
      double alfa = 0.0, weight = 0.0;

      switch (i) {
        case (0):
          switch (points) {
            case (2):
              alfa = -1.0 / sqrt(3.0);  // 2PTOS
              weight = 1.0;
              break;
            case (3):
              alfa = -0.774596669;  // 3PTOS
              weight = 5.0 / 9.0;
              break;
            case (4):
              alfa = -0.861136312;  // 4PTOS
              weight = 0.347854845;
              break;
            case (5):
              alfa = -0.9061798459;  // 5PTOS
              weight = 0.2369268850;
              break;
            case (6):
              alfa = -0.9324695142;  // 6PTOS
              weight = 0.1713244923;
              break;
            case (7):
              alfa = -0.9491079123;  // 7PTOS
              weight = 0.1294849661;
              break;
            case (8):
              alfa = -0.9602898564;  // 8PTOS
              weight = 0.1012285362;
              break;
          }
          break;
        case (1):
          switch (points) {
            case (2):
              alfa = 1.0 / sqrtf(3.0);  // 2PTOS
              weight = 1.0;
              break;
            case (3):
              alfa = 0.0;  // 3PTOS
              weight = 8.0 / 9.0;
              break;
            case (4):
              alfa = -0.339981043;  // 4PTOS
              weight = 0.652145155;
              break;
            case (5):
              alfa = -0.5384693101;  // 5PTOS
              weight = 0.4786286704;
              break;
            case (6):
              alfa = -0.6612093864;  // 6PTOS
              weight = 0.3607615730;
              break;
            case (7):
              alfa = -0.7415311855;  // 7PTOS
              weight = 0.2797053914;
              break;
            case (8):
              alfa = -0.7966664774;  // 8PTOS
              weight = 0.2223810344;
              break;
          }
          break;
        case (2):
          switch (points) {
            case (3):
              alfa = 0.774596669;  // 3PTOS
              weight = 5.0 / 9.0;
              break;
            case (4):
              alfa = 0.339981043;  // 4PTOS
              weight = 0.652145155;
              break;
            case (5):
              alfa = 0.0;  // 5PTOS
              weight = 0.5688888888;
              break;
            case (6):
              alfa = -0.2386191860;  // 6PTOS
              weight = 0.4679139345;
              break;
            case (7):
              alfa = -0.4058451513;  // 7PTOS
              weight = 0.3818300505;
              break;
            case (8):
              alfa = -0.5255324099;  // 8PTOS
              weight = 0.3137066458;
              break;
          }
          break;
        case (3):
          switch (points) {
            case (4):
              alfa = 0.861136312;  // 4PTOS
              weight = 0.347854845;
              break;
            case (5):
              alfa = 0.5384693101;  // 5PTOS
              weight = 0.4786286704;
              break;
            case (6):
              alfa = 0.2386191860;  // 6PTOS
              weight = 0.4679139345;
              break;
            case (7):
              alfa = 0.0;  // 7PTOS
              weight = 0.4179591836;
              break;
            case (8):
              alfa = -0.1834346424;  // 8PTOS
              weight = 0.3626837833;
              break;
          }
          break;
        case (4):
          switch (points) {
            case (5):
              alfa = 0.9061798459;  // 5PTOS
              weight = 0.2369268850;
              break;
            case (6):
              alfa = 0.6612093864;  // 6PTOS
              weight = 0.3607615730;
              break;
            case (7):
              alfa = 0.4058451513;  // 7PTOS
              weight = 0.3818300505;
              break;
            case (8):
              alfa = 0.1834346424;  // 8PTOS
              weight = 0.3626837833;
              break;
          }
          break;
        case (5):
          switch (points) {
            case (6):
              alfa = 0.9324695142;  // 6PTOS
              weight = 0.1713244923;
              break;
            case (7):
              alfa = 0.7415311855;  // 7PTOS
              weight = 0.2797053914;
              break;
            case (8):
              alfa = 0.5255324099;  // 8PTOS
              weight = 0.3137066458;
              break;
          }
          break;
        case (6):
          switch (points) {
            case (7):
              alfa = 0.9491079123;  // 7PTOS
              weight = 0.1294849661;
              break;
            case (8):
              alfa = 0.7966664774;  // 8PTOS
              weight = 0.2223810344;
              break;
          }
          break;
        case (7):
          alfa = 0.9602898564;  // 8PTOS
          weight = 0.1012285362;
          break;
      }

      double parameter_alfa = ((uf + ui) / 2.0) + ((uf - ui) / 2.0) * alfa;

      VectorAdaptive p = CalculateGradientByParameter(parameter_alfa);

      double f = p ^ p;

      localTotal += weight * sqrt(f);
    }

    localTotal *= (uf - ui) / 2.0;
    total += localTotal;
  }

  return total;
}

double CurveAdaptiveParametric::CalculateMidpointBisection(double parameter1,
                                                           double parameter2) {
  double a = parameter1 < parameter2 ? parameter1 : parameter2;
  double b = parameter1 < parameter2 ? parameter2 : parameter1;
  double l = CalculateParametricLength(a, b);
  // l - 2.0*comprimento(a, a);
  double yA = l;
  // l - 2.0*comprimento(a, b);
  double yB = -l;
  double half = 0.0;

  double newa = a;
  double newb = b;

  for (int i = 0; i < 5000; i++) {
    half = (newa + newb) / 2.0;

    if (fabs(newb - newa) < TOLERANCE) {
      return half;
    } else {
      double yH = l - 2.0 * CalculateParametricLength(a, half);

      if (fabs(yH) < TOLERANCE) {
        return half;
      }

      if (yA * yH < 0.0) {
        newb = half;
        yB = yH;
      } else if (yB * yH < 0.0) {
        newa = half;
        yA = yH;
      }
    }
  }

  return -1.0;
}
