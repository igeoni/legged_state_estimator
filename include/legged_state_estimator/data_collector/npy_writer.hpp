#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace legged_state_estimator {

/**
 * Minimal writer for NumPy .npy format (version 1.0).
 *
 * Supports float32 2-D arrays and int32 1-D arrays.
 * Compatible with np.load() without allow_pickle.
 */
class NpyWriter {
 public:
  // Save float32 matrix  (rows × cols)
  static void saveFloat32(const std::string& path,
                          const std::vector<float>& data,
                          std::size_t rows,
                          std::size_t cols);

  // Save int32 vector (length = rows)
  static void saveInt32(const std::string& path,
                        const std::vector<int32_t>& data,
                        std::size_t rows);

 private:
  static void writeHeader(std::ostream& out,
                          const std::string& dtype_str,
                          const std::string& shape_str);
};

}  // namespace legged_state_estimator
