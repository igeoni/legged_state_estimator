#include "legged_state_estimator/data_collector/npy_writer.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstring>

namespace legged_state_estimator {

// NumPy magic + version
static const char kMagic[]   = "\x93NUMPY";
static const char kVersion[] = "\x01\x00";

// Write header block padded to a multiple of 64 bytes (NPY v1.0 spec).
void NpyWriter::writeHeader(std::ostream& out,
                            const std::string& dtype_str,
                            const std::string& shape_str) {
  std::string dict = "{'descr': '" + dtype_str +
                     "', 'fortran_order': False, 'shape': (" + shape_str + "), }";

  // Preamble = 6 (magic) + 2 (version) + 2 (HEADER_LEN) = 10 bytes
  constexpr std::size_t preamble = 10;
  // We need: preamble + len(dict) + 1 (\n) ≡ 0 (mod 64)
  std::size_t raw_len = preamble + dict.size() + 1;
  std::size_t padded  = ((raw_len + 63) / 64) * 64;
  std::size_t padding = padded - raw_len;

  dict += std::string(padding, ' ');
  dict += '\n';

  auto header_len = static_cast<uint16_t>(dict.size());

  out.write(kMagic, 6);
  out.write(kVersion, 2);
  out.write(reinterpret_cast<const char*>(&header_len), sizeof(uint16_t));
  out.write(dict.c_str(), dict.size());
}

void NpyWriter::saveFloat32(const std::string& path,
                            const std::vector<float>& data,
                            std::size_t rows,
                            std::size_t cols) {
  if (data.size() != rows * cols) {
    throw std::runtime_error("NpyWriter: data.size() != rows * cols");
  }

  std::ofstream file(path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("NpyWriter: cannot open " + path);
  }

  std::string shape_str = std::to_string(rows) + ", " + std::to_string(cols);
  writeHeader(file, "<f4", shape_str);

  file.write(reinterpret_cast<const char*>(data.data()),
             static_cast<std::streamsize>(data.size() * sizeof(float)));
}

void NpyWriter::saveInt32(const std::string& path,
                          const std::vector<int32_t>& data,
                          std::size_t rows) {
  if (data.size() != rows) {
    throw std::runtime_error("NpyWriter: data.size() != rows");
  }

  std::ofstream file(path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("NpyWriter: cannot open " + path);
  }

  // 1-D shape: (rows,)
  std::string shape_str = std::to_string(rows) + ",";
  writeHeader(file, "<i4", shape_str);

  file.write(reinterpret_cast<const char*>(data.data()),
             static_cast<std::streamsize>(data.size() * sizeof(int32_t)));
}

}  // namespace legged_state_estimator
