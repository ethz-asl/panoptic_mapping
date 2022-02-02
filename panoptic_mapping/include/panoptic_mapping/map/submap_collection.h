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
#include "panoptic_mapping/map/tracked_instance_info.h"

namespace panoptic_mapping {

/***
 * @brief This class contains and manages access to all submaps. It represents
 * the full map consisting of all submaps.
 */
class SubmapCollection {
 public:
  // Construction.
  SubmapCollection() = default;
  SubmapCollection(SubmapCollection&&) = default;
  virtual ~SubmapCollection() = default;

  // IO.
  /**
   * @brief Save the current map to disk (.panmap file).
   *
   * @param file_path Filename including full path and extension to save to.
   * @return True if the map was saved successfully.
   */
  bool saveToFile(const std::string& file_path) const;

  /**
   * @brief Load a saved map (.panmap file) from disk, overwriting the current
   * content of the submap collection.
   *
   * @param file_path Filename including full path and extension to load.
   * @param recompute_data Whether to recompute all derived qualities (such as
   * meshes, bounding volumes, ...) for loaded map.
   * @return True if the map was loaded successfully.
   */
  bool loadFromFile(const std::string& file_path, bool recompute_data = true);

  // Modifying the collection.
  /**
   * @brief Create a new submap and add it to the collection. This is the only
   * way new submaps can be added.
   *
   * @param config Config of the submap to create.
   * @return Pointer to the newly created submap.
   */
  Submap* createSubmap(const Submap::Config& config);

  /**
   * @brief Remove a submap from the collection.
   *
   * @param id SubmapID of the submap to be deleted.
   * @return True if the submap existed and was deleted.
   */
  bool removeSubmap(int id);

  /**
   * @brief Remove all submaps contained in the collection. Also resets the
   * SubmapID and InstanceID trackers.
   */
  void clear();

  // Access.
  size_t size() const { return submaps_.size(); }
  /**
   * @brief Whether the submap of given ID exists.
   *
   * @param id SubmapID to check for.
   * @return True if SubmapID id exists.
   */
  bool submapIdExists(int id) const;

  /**
   * @brief Const access to the requested submap. Assumes that the provided id
   * exists, if unsure use 'submapIdExists(id)' first.
   *
   * @param id SubmapID to retrieve.
   */

  const Submap& getSubmap(int id) const;
  /**
   * @brief Modifying access to the requested submap. Assumes that the provided
   * id exists, if unsure use 'submapIdExists(id)' first.
   *
   * @param id SubmapID to retrieve.
   */
  Submap* getSubmapPtr(int id);

  int getActiveFreeSpaceSubmapID() const { return active_freespace_submap_id_; }
  const std::unordered_map<int, std::unordered_set<int>>&
  getInstanceToSubmapIDTable() const {
    return instance_to_submap_ids_;
  }

  const std::unordered_map<int, TrackedInstanceInfo>&
  getTrackedInstancesInfoTable() const {
    return tracked_instances_info_;
  }

  // Setters.
  void setActiveFreeSpaceSubmapID(int id) { active_freespace_submap_id_ = id; }
  void setIsSingleTsdf(bool is_single_tsdf) {
    is_single_tsdf_ = is_single_tsdf;
  }

  // Tools.

  // Utility function that tells you which submaps are new that are not in the
  // id list and which ones are deleted that were in the id list.
  void updateIDList(const std::vector<int>& id_list, std::vector<int>* new_ids,
                    std::vector<int>* deleted_ids) const;

  // Update the list of contained submaps for each instance.
  void updateInstanceToSubmapIDTable();

  void updateTrackedInstanceInfo(int instance_id, float instance_score,
                                 int class_id, float matching_score);

  // Creates a deep copy of all submaps, with new submap and instance id
  // managers. The submap ids may diverge when new submaps are added after
  // copying so be careful to manage these appropriately if information is
  // to be fused back to the original collection.
  std::unique_ptr<SubmapCollection> clone() const;

  /**
   * @brief Creates a filename that matches the map file extension (.panmap) if
   * that's not already the case.
   *
   * @param file Filename to check.
   * @return The new filename.
   */
  static std::string checkMapFileExtension(const std::string& file);

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

  // Used only in single TSDF mode
  bool is_single_tsdf_ = false;
  std::unordered_map<int, TrackedInstanceInfo> tracked_instances_info_;

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
