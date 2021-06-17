#ifndef PANOPTIC_MAPPING_MAP_SUBMAP_COLLECTION_H_
#define PANOPTIC_MAPPING_MAP_SUBMAP_COLLECTION_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap.h"

namespace panoptic_mapping {

/***
 * This class contains and manages access to all submaps.
 */
class SubmapCollection {
 public:
  /* Construction */
  SubmapCollection() = default;
  SubmapCollection(SubmapCollection&&) = default;
  virtual ~SubmapCollection() = default;

  /* IO */
  bool saveToFile(const std::string& file_path) const;
  bool loadFromFile(const std::string& file_path, bool recompute_data = true);

  /* Modify the collection */
  Submap* createSubmap(const Submap::Config& config);
  bool removeSubmap(int id);
  void clear();

  /* Accessors */
  size_t size() const { return submaps_.size(); }
  bool submapIdExists(int id) const;      // Check whether id exists.
  const Submap& getSubmap(int id) const;  // This assumes that the id exists.
  Submap* getSubmapPtr(int id);           // This assumes that the id exists.
  int getActiveFreeSpaceSubmapID() const { return active_freespace_submap_id_; }
  const std::unordered_map<int, std::unordered_set<int>>&
  getInstanceToSubmapIDTable() const {
    return instance_to_submap_ids_;
  }

  /* Setters */
  void setActiveFreeSpaceSubmapID(int id) { active_freespace_submap_id_ = id; }

  /* Tools */
  // Utility function that tells you which submaps are new that are not in the
  // id list and which ones are deleted that were in the id list.
  void updateIDList(const std::vector<int>& id_list, std::vector<int>* new_ids,
                    std::vector<int>* deleted_ids) const;

  // Update the list of contained submaps for each instance.
  void updateInstanceToSubmapIDTable();

  // Creates a deep copy of all submaps, with new submap and instance id
  // managers. The submap ids may diverge when new submaps are added after
  // copying so be careful to manage these appropriately if information is to be
  // fused back to the original collection.
  std::unique_ptr<SubmapCollection> clone() const;

 private:
  // IDs are managed within a submap collection.
  SubmapIDManager submap_id_manager_;
  InstanceIDManager instance_id_manager_;

  // The map.
  std::vector<std::unique_ptr<Submap>> submaps_;

  // Bookkeeping.
  std::unordered_map<int, size_t> id_to_index_;
  std::unordered_map<int, std::unordered_set<int>> instance_to_submap_ids_;
  int active_freespace_submap_id_ = -1;

 public:
  // Iterators over submaps.
  class iterator : public std::iterator<std::random_access_iterator_tag, Submap,
                                        size_t, Submap*, Submap&> {
   public:
    explicit iterator(size_t index = 0,
                      std::vector<std::unique_ptr<Submap>>* data = nullptr)
        : index_(index), data_(data) {}
    iterator& operator++() {
      index_++;
      return *this;
    }
    iterator operator++(int) {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(iterator other) const {
      return index_ == other.index_ && data_ == other.data_;
    }
    bool operator!=(iterator other) const { return !(*this == other); }
    reference operator*() const { return *(data_->at(index_)); }
    pointer operator->() const { return data_->at(index_).get(); }

   private:
    std::vector<std::unique_ptr<Submap>>* const data_;
    size_t index_;
  };

  class const_iterator
      : public std::iterator<std::random_access_iterator_tag, const Submap,
                             size_t, Submap const*, const Submap&> {
   public:
    explicit const_iterator(
        size_t index = 0,
        const std::vector<std::unique_ptr<Submap>>* data = nullptr)
        : index_(index), data_(data) {}
    const_iterator& operator++() {
      index_++;
      return *this;
    }
    const_iterator operator++(int) {
      const_iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(const_iterator other) const {
      return index_ == other.index_ && data_ == other.data_;
    }
    bool operator!=(const_iterator other) const { return !(*this == other); }
    reference operator*() const { return *(data_->at(index_)); }
    pointer operator->() const { return data_->at(index_).get(); }

   private:
    const std::vector<std::unique_ptr<Submap>>* const data_;
    size_t index_;
  };

  iterator begin() { return iterator(0, &submaps_); }
  iterator end() { return iterator(submaps_.size(), &submaps_); }
  const_iterator begin() const { return const_iterator(0, &submaps_); }
  const_iterator end() const {
    return const_iterator(submaps_.size(), &submaps_);
  }
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_SUBMAP_COLLECTION_H_
