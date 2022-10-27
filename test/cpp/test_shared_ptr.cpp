#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <string>

struct data {
  std::vector<double> values;
  int id;
};

std::unordered_map<int, std::shared_ptr<data>> data_idlookup;
std::vector<std::shared_ptr<data>> feats_selected;

void printData( const char *str) {
  printf("\n------------ %s ---------------\n", str);

  // show original data
  printf("\noriginal data: \n");
  for(const auto& pair : data_idlookup){
    printf(" id: %d", pair.first);
    for(const auto& i :pair.second->values) {
      printf(": %f",i);
    }
    printf("\n");
  }
  printf("\n");

  // show selected data
  printf("selected: \n");
  for(const auto& i : feats_selected[0]->values){
    std::cout<<" : "<<i;
  }
  printf("\n");
}

void addToSelect() {
  feats_selected[0]->values.push_back(13.1);
}

void deleteOrigWhole() {
  // delete original whole data
  data_idlookup.erase(1);
}

void deleteOrigPartial() {
  data_idlookup[1]->values.erase(data_idlookup[1]->values.begin(),data_idlookup[1]->values.begin()+2);
}

int main() {
  // feature 0
  std::shared_ptr<data> feat0 = std::make_shared<data>();
  feat0->values.push_back(32.1);
  feat0->values.push_back(3.31);
  feat0->values.push_back(-0.31);
  feat0->id = 0;
  // feature 1
  std::shared_ptr<data> feat1 = std::make_shared<data>();
  feat1->values.push_back(2.1);
  feat1->values.push_back(31);
  feat1->values.push_back(0.99);
  feat1->id = 1;

  // original data
  data_idlookup[feat0->id] = feat0;
  data_idlookup[feat1->id] = feat1;

  // select data
  feats_selected.push_back(data_idlookup[1]);

  printData("Initial");

  // // add to selected one
  // addToSelect();
  // printData("Added to Selected");

  // delete whole
  deleteOrigWhole();
  printData("Delete original whole");

  // // delete partial 
  // deleteOrigPartial();
  // printData("Delete original partial");

}