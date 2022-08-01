#include <iostream>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <memory>

namespace test{


class Feature {

public:
  Feature(){}

  size_t id;

  bool to_delete;

  std::vector<double> values;

};


class Database {

public:
  Database(){}

  void update_feature(size_t id, double value) {
    // Find this feature using the ID lookup
    std::unique_lock<std::mutex> lck(mtx);

    if (features_idlookup.find(id) != features_idlookup.end()) {
      // Get our feature
      std::shared_ptr<Feature> feat = features_idlookup.at(id);
      // Append this new information to it!
      feat->values.emplace_back(value);
      return;
    }

    // Else we have not found the feature, so lets make it be a new one!
    std::shared_ptr<Feature> feat = std::make_shared<Feature>();
    feat->id = id;
    feat->values.emplace_back(value);

    // Append this new feature into our database
    features_idlookup[id] = feat;
  }

  std::vector<std::shared_ptr<Feature>> getFeature(size_t given_id) {
    std::vector<std::shared_ptr<Feature>> feats_selected;

    std::unique_lock<std::mutex> lck(mtx);

    // for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
    //   if(it->second->id <= given_id){
    //     feats_selected.emplace_back(it->second);
    //     it++;
    //   }
    //   else{
    //     break;
    //   }
    // }

    for(const auto &feat: features_idlookup) {
      if(feat.second->id <= given_id) {
        feats_selected.emplace_back(feat.second);
      }
    }

    return feats_selected;
  }

  void showFeature(){
    std::unique_lock<std::mutex> lck(mtx);

    for(const auto &feat: features_idlookup) {
      printf("feat id: %ld\n", feat.second->id);
      for(const auto &f : feat.second->values) {
        printf("  value: %f\n", f);
      }
    }
    printf("----------\n");
  }

  void cleanup() {
    std::unique_lock<std::mutex> lck(mtx);

    for (auto it = features_idlookup.begin(); it != features_idlookup.end();) {
      // If delete flag is set, then delete it
      // NOTE: if we are using a shared pointer, then no need to do this!
      if ((*it).second->to_delete) {
        // delete (*it).second;
        features_idlookup.erase(it++);
      } else {
        it++;
      }
    }
  }

  void removeFeature(size_t id) {
    std::unique_lock<std::mutex> lck(mtx);

    features_idlookup.erase(id);
  }

  std::mutex mtx;

  std::unordered_map<size_t, std::shared_ptr<Feature>> features_idlookup;
};

}

using namespace test;


// database 
//   - feature_0[0.3, 1.3, 4,2] x
//   - feature_1[9.3, 3.3, 2,3]
//   - feature_0[2.1]

// selected:
//   - feature_0[0.3, 1.3, 4,2]

int main() {

  //// create feature database
  printf("\n create features in database:\n");
  std::shared_ptr<Database> database = std::make_shared<Database>();
  database->update_feature(0, 2.1);
  database->update_feature(1, 0.1);
  database->update_feature(2, 9.12);
  database->update_feature(3, 4.21);
  database->update_feature(4, 53.1);
  database->showFeature();

  //// selected features from database
  std::vector<std::shared_ptr<Feature>> selected_features = database->getFeature(3);
  printf("\n show features in selected features:\n");
  for(const auto & selected: selected_features) {
    printf("selected id: %ld\n", selected->id);
    for(const auto &f: selected->values){
      printf("  feature:%f\n", f);
    }
  }

  //// delete feature 3
  printf("\n show features database after remove:\n");
  database->removeFeature(3);
  database->showFeature();


  //// update selected features_3
  for(auto & f: selected_features) {
    if(f->id == 3){
      f->values.push_back(4.14);
    }
  }

  //// check selected features
  printf("\n show selected features after update:\n");
  for(const auto & selected: selected_features) {
    printf("selected id: %ld\n", selected->id);
    for(const auto &f: selected->values){
      printf("  feature:%f\n", f);
    }
  }

  //// update feature database
  printf("\n show the features database after update:\n");
  database->showFeature();

}

  ///////////////////////////////////////////////////////////////////////////////////////
  // 1) get features_3 from database
  // 2) remove feature_3 from database
  // 3) update features_3 in database
  // 4) selected feature_3 not changed 
  ///////////////////////////////////////////////////////////////////////////////////////

  // //// create feature database
  // printf("\n create features in database:\n");
  // std::shared_ptr<Database> database = std::make_shared<Database>();
  // database->update_feature(0, 2.1);
  // database->update_feature(1, 0.1);
  // database->update_feature(2, 9.12);
  // database->update_feature(3, 4.21);
  // database->update_feature(4, 53.1);
  // database->showFeature();

  // //// selected features from database
  // std::vector<std::shared_ptr<Feature>> selected_features = database->getFeature(3);
  // printf("\n show features in selected features:\n");
  // for(const auto & selected: selected_features) {
  //   printf("selected id: %ld\n", selected->id);
  //   for(const auto &f: selected->values){
  //     printf("  feature:%f\n", f);
  //   }
  // }

  // //// delete feature 3
  // printf("\n show features database after remove:\n");
  // database->removeFeature(3);
  // database->showFeature();


  // //// check selected features
  // printf("\n show selected features after remove:\n");
  // for(const auto & selected: selected_features) {
  //   printf("selected id: %ld\n", selected->id);
  //   for(const auto &f: selected->values){
  //     printf("  feature:%f\n", f);
  //   }
  // }

  // //// update feature database
  // printf("\n show the features database after update:\n");
  // database->update_feature(3, 0.3);
  // database->showFeature();


  // //// check selected features
  // printf("\n show selected features after update:\n");
  // for(const auto & selected: selected_features) {
  //   printf("selected id: %ld\n", selected->id);
  //   for(const auto &f: selected->values){
  //     printf("  feature:%f\n", f);
  //   }
  // }