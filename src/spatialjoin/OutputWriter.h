// Copyright 2025, University of Freiburg
// Authors: Patrick Brosi <brosi@cs.uni-freiburg.de>.

#ifndef SPATIALJOINS_OUTPUTWRITER_H_
#define SPATIALJOINS_OUTPUTWRITER_H_

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef SPATIALJOIN_NO_BZIP2
#include <bzlib.h>
#endif

#ifndef SPATIALJOIN_NO_ZLIB
#include <zlib.h>
#endif

namespace sj {

static const size_t BUFFER_S_PAIRS = 1024 * 1024 * 10;

enum OutMode : uint8_t { PLAIN = 0, BZ2 = 1, GZ = 2, COUT = 3, NONE = 4 };

inline uint64_t intFromString(const char* s, size_t n) {
  uint64_t id = 0;

  for (size_t i = n; i > 0; i--) {
    id |= static_cast<uint64_t>(static_cast<unsigned char>(s[i - 1]))
          << (8 * (n - 1 - (i - 1)));
  }

  return id;
}

class OutputWriter {
 public:
  ~OutputWriter() {
    flushOutputFiles();
    for (size_t i = 0; i < _outBuffers.size(); i++) {
      if (_outBuffers[i]) delete[] _outBuffers[i];
    }
  }

  OutputWriter(size_t numThreads, const std::string& prefix,
               const std::string& suffix, const std::string& out,
               const std::string& cache)
      : _numThreads(numThreads),
        _prefix(prefix),
        _suffix(suffix),
        _out(out),
        _cache(cache) {
    if (util::endsWith(out, ".bz2")) {
#ifndef SPATIALJOIN_NO_BZIP2
      _outMode = BZ2;
#else
      throw std::runtime_error("spatialjoin was compiled without BZ2 support");
#endif
    } else if (util::endsWith(out, ".gz")) {
#ifndef SPATIALJOIN_NO_ZLIB
      _outMode = GZ;
#else
      throw std::runtime_error("spatialjoin was compiled without GZ support");
#endif
    } else if (out.size()) {
      _outMode = PLAIN;
    } else {
      struct stat std_out;
      struct stat dev_null;
      if (fstat(STDOUT_FILENO, &std_out) == 0 && S_ISCHR(std_out.st_mode) &&
          stat("/dev/null", &dev_null) == 0 &&
          std_out.st_dev == dev_null.st_dev &&
          std_out.st_ino == dev_null.st_ino) {
        // output to /dev/null, print nothing
        _outMode = NONE;
      } else {
        _outMode = COUT;
      }
    }

    prepareOutputFiles();
  }

  OutMode getOutMode() const { return _outMode; }

  void flushOutputFiles() {
    if (_outMode == COUT) {
      for (size_t i = 0; i < _numThreads + 1; i++) {
        _outBuffers[i][_outBufPos[i]] = 0;
        fputs(reinterpret_cast<const char*>(_outBuffers[i]), stdout);
      }
    }
    if (_outMode == BZ2 || _outMode == GZ || _outMode == PLAIN) {
      if (_outMode == BZ2) {
#ifndef SPATIALJOIN_NO_BZIP2
        for (size_t i = 0; i < _numThreads + 1; i++) {
          int err = 0;
          BZ2_bzWrite(&err, _bzFiles[i], _outBuffers[i], _outBufPos[i]);
          if (err == BZ_IO_ERROR) {
            BZ2_bzWriteClose(&err, _bzFiles[i], 0, 0, 0);
            std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                                "-" + std::to_string(i);
            std::stringstream ss;
            ss << "Could not write spatial relation to temporary bzip2 file '"
               << fname << "':\n";
            ss << strerror(errno) << std::endl;
            throw std::runtime_error(ss.str());
          }
          BZ2_bzWriteClose(&err, _bzFiles[i], 0, 0, 0);
          fclose(_rawFiles[i]);
        }
#endif
      } else if (_outMode == GZ) {
#ifndef SPATIALJOIN_NO_ZLIB
        for (size_t i = 0; i < _numThreads + 1; i++) {
          int r = gzwrite(_gzFiles[i], _outBuffers[i], _outBufPos[i]);
          if (r != (int)_outBufPos[i]) {
            gzclose(_gzFiles[i]);
            std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                                "-" + std::to_string(i);
            std::stringstream ss;
            ss << "Could not write spatial relation to temporary gzip file '"
               << fname << "':\n";
            ss << strerror(errno) << std::endl;
            throw std::runtime_error(ss.str());
          }
          gzclose(_gzFiles[i]);
        }
#endif
      } else {
        for (size_t i = 0; i < _numThreads + 1; i++) {
          size_t r =
              fwrite(_outBuffers[i], sizeof(char), _outBufPos[i], _rawFiles[i]);
          if (r != _outBufPos[i]) {
            std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                                "-" + std::to_string(i);
            std::stringstream ss;
            ss << "Could not write spatial relation to temporary file '"
               << fname << "':\n";
            ss << strerror(errno) << std::endl;
            throw std::runtime_error(ss.str());
          }
          fclose(_rawFiles[i]);
        }
      }

      // merge files into first file
      std::ofstream out(
          _cache + "/.rels" + std::to_string(getpid()) + "-0",
          std::ios_base::binary | std::ios_base::app | std::ios_base::ate);
      for (size_t i = 1; i < _numThreads + 1; i++) {
        std::string fName = _cache + "/.rels" + std::to_string(getpid()) + "-" +
                            std::to_string(i);
        std::ifstream ifCur(fName, std::ios_base::binary);
        if (ifCur.peek() != std::ifstream::traits_type::eof())
          out << ifCur.rdbuf();
        std::remove(fName.c_str());
      }

      // move first file to output file
      std::rename((_cache + "/.rels" + std::to_string(getpid()) + "-0").c_str(),
                  _out.c_str());
    }
  }

  void prepareOutputFiles() {
    _rawFiles = {};

#ifndef SPATIALJOIN_NO_BZIP2
    _bzFiles = {};
    _bzFiles.resize(_numThreads + 1);
#endif

#ifndef SPATIALJOIN_NO_ZLIB
    _gzFiles = {};
    _gzFiles.resize(_numThreads + 1);
#endif

    _outBufPos = {};
    _outBuffers = {};

    _rawFiles.resize(_numThreads + 1);
    _outBufPos.resize(_numThreads + 1);
    _outBuffers.resize(_numThreads + 1);

    if (_outMode == BZ2) {
#ifndef SPATIALJOIN_NO_BZIP2
      for (size_t i = 0; i < _numThreads + 1; i++) {
        std::string fname = _cache + "/.rels" + std::to_string(getpid()) + "-" +
                            std::to_string(i);
        _rawFiles[i] = fopen(fname.c_str(), "w");

        if (_rawFiles[i] == NULL) {
          std::stringstream ss;
          ss << "Could not open temporary bzip2 file '" << fname
             << "' for writing:\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }

        int err = 0;
        _bzFiles[i] = BZ2_bzWriteOpen(&err, _rawFiles[i], 6, 0, 30);
        if (err != BZ_OK) {
          std::stringstream ss;
          ss << "Could not open temporary bzip2 file '" << fname
             << "' for writing:\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }
        _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
      }
#endif
    } else if (_outMode == GZ) {
#ifndef SPATIALJOIN_NO_ZLIB
      for (size_t i = 0; i < _numThreads + 1; i++) {
        std::string fname = _cache + "/.rels" + std::to_string(getpid()) + "-" +
                            std::to_string(i);
        _gzFiles[i] = gzopen(fname.c_str(), "w");

        if (_gzFiles[i] == Z_NULL) {
          std::stringstream ss;
          ss << "Could not open temporary gzip file '" << fname
             << "' for writing:\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }
        _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
      }
#endif
    } else if (_outMode == PLAIN) {
      for (size_t i = 0; i < _numThreads + 1; i++) {
        std::string fname = _cache + "/.rels" + std::to_string(getpid()) + "-" +
                            std::to_string(i);
        _rawFiles[i] = fopen(fname.c_str(), "w");

        if (_rawFiles[i] == NULL) {
          std::stringstream ss;
          ss << "Could not open temporary file '" << fname
             << "' for writing:\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }

        _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
      }
    } else if (_outMode == COUT) {
      for (size_t i = 0; i < _numThreads + 1; i++) {
        _outBuffers[i] = new unsigned char[BUFFER_S_PAIRS];
      }
    }
  }

  void writeRelToBuf(size_t t, const char* a, size_t an, const char* b, size_t bn,
                     const char* pred, size_t predn) {
    memcpy(_outBuffers[t] + _outBufPos[t], _prefix.c_str(), _prefix.size());
    _outBufPos[t] += _prefix.size();
    memcpy(_outBuffers[t] + _outBufPos[t], a, an);
    _outBufPos[t] += an;
    memcpy(_outBuffers[t] + _outBufPos[t], pred, predn);
    _outBufPos[t] += predn;
    memcpy(_outBuffers[t] + _outBufPos[t], b, bn);
    _outBufPos[t] += bn;
    memcpy(_outBuffers[t] + _outBufPos[t], _suffix.c_str(), _suffix.size());
    _outBufPos[t] += _suffix.size();
  }

  void writeRelCb(size_t t, const char* a, size_t an, const char* b, size_t bn,
                  const char* pred, size_t predn) {
    std::string tmpa, tmpb;

    if (an > 0 && a[0] == 's') {
      a = &a[1];
      an--;
    } else if (an > 0 && a[0] == 'd') {
      tmpa = std::to_string(intFromString(&a[1], an - 1));
      a = tmpa.c_str();
      an = tmpa.size();
    }

    if (bn > 0 && b[0] == 's') {
      b = &b[1];
      bn--;
    } else if (bn > 0 && b[0] == 'd') {
      tmpb = std::to_string(intFromString(&b[1], bn - 1));
      b = tmpb.c_str();
      bn = tmpb.size();
    }

    size_t totSize = _prefix.size() + an + predn + bn + _suffix.size();

    if (_outMode == BZ2) {
#ifndef SPATIALJOIN_NO_BZIP2
      if (_outBufPos[t] + totSize >= BUFFER_S_PAIRS) {
        int err = 0;
        BZ2_bzWrite(&err, _bzFiles[t], _outBuffers[t], _outBufPos[t]);
        if (err == BZ_IO_ERROR) {
          BZ2_bzWriteClose(&err, _bzFiles[t], 0, 0, 0);
          std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                              "-" + std::to_string(t);
          std::stringstream ss;
          ss << "Could not write spatial relation to temporary bzip2 file '"
             << fname << "':\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }
        _outBufPos[t] = 0;
      }

      writeRelToBuf(t, a, an, b, bn, pred, predn);
#endif
    } else if (_outMode == GZ) {
#ifndef SPATIALJOIN_NO_ZLIB
      if (_outBufPos[t] + totSize >= BUFFER_S_PAIRS) {
        int r = gzwrite(_gzFiles[t], _outBuffers[t], _outBufPos[t]);
        if (r != (int)_outBufPos[t]) {
          gzclose(_gzFiles[t]);
          std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                              "-" + std::to_string(t);
          std::stringstream ss;
          ss << "Could not write spatial relation to temporary gzip file '"
             << fname << "':\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }
        _outBufPos[t] = 0;
      }

      writeRelToBuf(t, a, an, b, bn, pred, predn);
#endif
    } else if (_outMode == PLAIN) {
      if (_outBufPos[t] + totSize >= BUFFER_S_PAIRS) {
        size_t r =
            fwrite(_outBuffers[t], sizeof(char), _outBufPos[t], _rawFiles[t]);
        if (r != _outBufPos[t]) {
          std::string fname = _cache + "/.rels" + std::to_string(getpid()) +
                              "-" + std::to_string(t);
          std::stringstream ss;
          ss << "Could not write spatial relation to temporary file '" << fname
             << "':\n";
          ss << strerror(errno) << std::endl;
          throw std::runtime_error(ss.str());
        }
        _outBufPos[t] = 0;
      }

      writeRelToBuf(t, a, an, b, bn, pred, predn);
    } else if (_outMode == COUT) {
      if (_outBufPos[t] + totSize + 1 >= BUFFER_S_PAIRS) {
        _outBuffers[t][_outBufPos[t]] = 0;
        fputs(reinterpret_cast<const char*>(_outBuffers[t]), stdout);
        _outBufPos[t] = 0;
      }
      writeRelToBuf(t, a, an, b, bn, pred, predn);
    }
  }

 private:
  OutMode _outMode;
  size_t _numThreads;

  std::string _prefix;
  std::string _suffix;

  std::vector<FILE*> _rawFiles;

#ifndef SPATIALJOIN_NO_BZIP2
  std::vector<BZFILE*> _bzFiles;
#endif
#ifndef SPATIALJOIN_NO_ZLIB
  std::vector<gzFile> _gzFiles;
#endif
  std::vector<size_t> _outBufPos;
  std::vector<unsigned char*> _outBuffers;

  std::string _out, _cache;
};

}  // namespace sj
   //
#endif
