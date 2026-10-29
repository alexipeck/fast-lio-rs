// Minimal JSON writer for golden data. Doubles are printed with 17 significant
// digits so they round-trip exactly; matrices are flattened row-major.
#pragma once

#include <cstdio>
#include <unistd.h>

namespace json {

class Writer {
 public:
  explicit Writer(std::FILE *out) : out_(out) {}
  void reset(std::FILE *out) { out_ = out; }

  void begin_object() { sep(); std::fputc('{', out_); first_ = true; }
  void end_object() { std::fputc('}', out_); first_ = false; }
  void begin_array() { sep(); std::fputc('[', out_); first_ = true; }
  void end_array() { std::fputc(']', out_); first_ = false; }

  void key(const char *k) { sep(); std::fprintf(out_, "\"%s\":", k); first_ = true; }
  void num(double v) { sep(); std::fprintf(out_, "%.17g", v); }
  void boolean(bool v) { sep(); std::fputs(v ? "true" : "false", out_); }
  // Caller guarantees `s` needs no escaping.
  void str(const char *s) { sep(); std::fprintf(out_, "\"%s\"", s); }

  template <class M>
  void matrix(const M &m) {
    begin_array();
    for (int r = 0; r < m.rows(); ++r)
      for (int c = 0; c < m.cols(); ++c) num(static_cast<double>(m(r, c)));
    end_array();
  }

  void values(const double *p, int n) {
    begin_array();
    for (int i = 0; i < n; ++i) num(p[i]);
    end_array();
  }

 private:
  void sep() {
    if (!first_) std::fputc(',', out_);
    first_ = false;
  }

  std::FILE *out_;
  bool first_ = true;
};

// Returns a stream on the original stdout and points stdout at stderr, so
// stray printf output from FAST-LIO code (ikd-Tree's thread messages) cannot
// end up inside the JSON.
inline std::FILE *take_stdout() {
  std::fflush(stdout);
  std::FILE *json = fdopen(dup(STDOUT_FILENO), "w");
  dup2(STDERR_FILENO, STDOUT_FILENO);
  return json;
}

}  // namespace json
