#include "../../include/curvature/curvature_discrete.h"

// Kim, S. J., Jeong, W. K. e Kim, C. H. LOD generation with discrete curva-
// ture error metric. Em Proceedings of 2nd Korea Israel Bi-National Conference
// on Geometrical Modeling and Computer Graphics in the WWW Era (1999), pp.
// 97–104.

CurvatureDiscrete::CurvatureDiscrete(const NodeAdaptive& noh) {
  // O construtor ordena a lista de elementos do nó, por garantia...
  // para cada o elemento da lista de adjacências de n
  //		some sua área ao A deste objeto
  //		GetAngle ( n ) e some a sum_phi

  //	cout << "********** Curvatura discreta **********" << endl;

  //	cout << "ordenaAdjacência" << endl;

  this->AdjacencySort(noh);

  this->a_ = 0;
  this->sum_phi_ = 0;

  std::list<ElementAdaptive*>::const_iterator element_iterator;

  //	cout << "for ( ite = n.guarda_chuva.begin ( ); ite != n.guarda_chuva.end
  //( ); ++ite )\n{" << endl;

  for (element_iterator = noh.GetElements().begin();
       element_iterator != noh.GetElements().end(); ++element_iterator) {
    //		cout << "\tA += " << (*ite)->getArea( ) << endl;
    this->a_ += (*element_iterator)->GetArea();
    //		cout << "\tA == " << this->A << endl;
    //		cout << "\tsum_phi += " << (*ite)->GetAngle ( n ) << endl;
    this->sum_phi_ += (*element_iterator)->GetAngle(noh);
    //		cout << "\tsum_phi == " << this->sum_phi << endl;
  }

  this->noh_ = noh;

  //	cout << "******************************" << endl;
}

double CurvatureDiscrete::CalculateMeanCurvature() {
  double angle_gama = 0;  // ângulo entre dois elementos adjacentes

  while (this->elements_.size() > 1) {
    ElementAdaptive* first_element = this->elements_.front();
    this->elements_.pop_front();
    ElementAdaptive* next_element = this->elements_.front();

    // verifica concavidade entre dois elementos adjacentes
    const int concavity_value =
        adjacent_.ConcavityElement(this->noh_, *first_element, *next_element);

    // retorna o ângulo entre dois elementos adjacentes
    if (concavity_value) {
      angle_gama += concavity_value *
                    adjacent_.AngleElement(*first_element, *next_element);
    }
  }

  this->elements_.pop_front();  // esvazia a lista de elementos
  return 3.0 * angle_gama / this->a_;
}

double CurvatureDiscrete::CalculateGaussCurvature() {
  return (3.0 * (this->factor_disc_ - this->sum_phi_) / this->a_);
}

void CurvatureDiscrete::AdjacencySort(const NodeAdaptive& noh) {
  this->elements_.clear();

  // copiando os elementos da adjacência de n para a lista de elementos desta
  // classe
  //		copie a lista n.guarda_cuva para nova_lista
  std::list<ElementAdaptive*> new_list_elements;
  new_list_elements = noh.GetElements();

  //		retire o primeiro elemento E da nova_lista e insira em elementos
  ElementAdaptive* element_front = new_list_elements.front();
  this->elements_.push_back(element_front);
  new_list_elements.pop_front();

  //		enquanto encontrar um elemento adjacente à esquerda de E faça
  //			retire E' de nova_lista e insira ao final de elementos
  //			E recebe E'
  bool find_adj_left = true;  // encontrou um adjacente à esquerda

  while (find_adj_left) {
    ElementAdaptive* element =
        adjacent_.GetElementLeft(noh, element_front, new_list_elements);
    if (element) {
      this->elements_.push_back(element);
      element_front = element;
    } else
      find_adj_left = false;
  }

  //		se nova_lista não estiver vazia
  //			fator = M_PI ( n está na borda! )
  //			E recebe primeiro elemento da lista elementos ( sem
  // retirar
  //) 			enquanto encontrar um elemento adjacente à direita de E
  // faça

  //				retire E' de nova_lista e insira no início de
  // elementos 				E recebe E'
  if (!new_list_elements.empty()) {
    this->factor_disc_ = M_PI;  // n está na borda !!!
    element_front = this->elements_.front();

    bool find_adj_right = true;  // encontrou um adjacente à direita

    while (find_adj_right) {
      ElementAdaptive* element =
          adjacent_.GetElementRight(noh, element_front, new_list_elements);
      if (element) {
        this->elements_.push_front(element);
        element_front = element;
      } else
        find_adj_right = false;
    }
  }
  //		senão
  //			se o primeiro elemento P de elementos for adjacente ao
  // último U 				fator = 2*M_PI ( n está no interior )
  // senão 				fator = M_PI ( n está na borda
  //)
  else {
    if (adjacent_.ConfirmRightAdjacency(noh, *(this->elements_.front()),
                                        *(this->elements_.back()))) {
      this->factor_disc_ = 2 * M_PI;  // n está no interior !!!
      this->elements_.push_back(this->elements_.front());  // lista "circular"
    } else
      this->factor_disc_ = M_PI;  // n está no interior !!!
  }
}
