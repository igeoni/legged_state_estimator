#include "legged_state_estimator/data_buffer.hpp"
#include "legged_state_estimator/npy_writer.hpp"

#include <algorithm>
#include <numeric>
#include <random>
#include <stdexcept>
#include <filesystem>

namespace legged_state_estimator {

DataBuffer::DataBuffer(const std::string& output_dir)
    : output_dir_(output_dir), ratio_(SplitRatio{}) {
  std::filesystem::create_directories(output_dir_);
}

DataBuffer::DataBuffer(const std::string& output_dir, const SplitRatio& ratio)
    : output_dir_(output_dir), ratio_(ratio) {
  std::filesystem::create_directories(output_dir_);
}

void DataBuffer::addSample(const std::vector<float>& feature, int32_t label) {
  if (static_cast<int>(feature.size()) != kFeatureDim) {
    throw std::runtime_error("DataBuffer: wrong feature dimension");
  }
  features_.insert(features_.end(), feature.begin(), feature.end());
  labels_.push_back(label);
}

void DataBuffer::save() const {
  const std::size_t n = labels_.size();
  if (n == 0) return;

  // Shuffle indices
  std::vector<std::size_t> idx(n);
  std::iota(idx.begin(), idx.end(), 0);
  std::mt19937 rng(42);
  std::shuffle(idx.begin(), idx.end(), rng);

  std::size_t n_train = static_cast<std::size_t>(ratio_.train * n);
  std::size_t n_val   = static_cast<std::size_t>(ratio_.val   * n);

  std::vector<std::size_t> train_idx(idx.begin(),             idx.begin() + n_train);
  std::vector<std::size_t> val_idx  (idx.begin() + n_train,   idx.begin() + n_train + n_val);
  std::vector<std::size_t> test_idx (idx.begin() + n_train + n_val, idx.end());

  saveSplit("train", train_idx);
  saveSplit("val",   val_idx);
  saveSplit("test",  test_idx);
}

void DataBuffer::saveSplit(const std::string& prefix,
                           const std::vector<std::size_t>& indices) const {
  const std::size_t n = indices.size();
  if (n == 0) return;

  std::vector<float>   feat_out;
  std::vector<int32_t> label_out;
  feat_out.reserve(n * kFeatureDim);
  label_out.reserve(n);

  for (std::size_t idx : indices) {
    const float* row = features_.data() + idx * kFeatureDim;
    feat_out.insert(feat_out.end(), row, row + kFeatureDim);
    label_out.push_back(labels_[idx]);
  }

  NpyWriter::saveFloat32(output_dir_ + "/" + prefix + ".npy",
                         feat_out, n, kFeatureDim);
  NpyWriter::saveInt32  (output_dir_ + "/" + prefix + "_label.npy",
                         label_out, n);
}

}  // namespace legged_state_estimator
