#ifndef __COMMON_BACKTRACE_H__
#define __COMMON_BACKTRACE_H__

#include <execinfo.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <iostream>
#include <fstream>
#include <cxxabi.h>
//#include "Common/Debug_arm.h"
namespace deb {
    class Backtrace {
    public:
        Backtrace() {
            _traceDepth = ::backtrace(_trace, MAX_BACKTRACE_DEPTH);
        }

        size_t traceDepth() const {
            return _traceDepth;
        }

        const void *traceLine(unsigned int line) const {
            return _trace[line];
        }

        void backtraceToStream(FILE *stream) const {
            fflush(stream);
            backtrace_symbols_fd(_trace, _traceDepth, fileno(stream));
        }

        friend std::ostream &operator<<(std::ostream &stream, const Backtrace &backtrace) {
            char **strings = backtrace_symbols(backtrace._trace, backtrace._traceDepth);
            stream << "Backtrace: " << backtrace.hash() << "\n";
            for (unsigned i = 0; i < backtrace._traceDepth; i++)
                stream << convertToHumanReadableFormat(i, strings[i]) << "\n";
            free(strings);
            return stream;
        }

#define DISCARD_RETURN(x) do {auto ret = (x); (void)ret;} while(0)

        std::string addr2Line() {
            std::string filename = std::string("/tmp/") + hash() + ".backtrace";
            for (unsigned i = 0; i < _traceDepth; i++) {
                auto info = symbolInfo(_trace[i]);
                if (info.dli_fname == nullptr) {
                    std::cout << "Unable to get symbol info for " << _trace[i] << std::endl;
                    continue;
                }
                char relative[16];
                sprintf(relative, "0x%lX", static_cast<unsigned long>(
                        reinterpret_cast<unsigned char *>(_trace[i]) -
                        reinterpret_cast<unsigned char *>(info.dli_fbase)));
                DISCARD_RETURN(::system((std::string("addr2line -e ") + info.dli_fname + " " + relative +
                                         " >" + ((i == 0) ? "" : ">") + filename).c_str()));
            }
            std::string allLines(readTextFile(std::move(filename)));
            ::unlink(filename.c_str());
            return hash() + ":\n" + allLines;
        }

    private:
        static const size_t MAX_BACKTRACE_DEPTH = 20;

        static std::string convertToHumanReadableFormat(unsigned ind, const char *symbol) {
            std::string line = std::to_string(ind) + ") ";
            unsigned begin, end;
            for (begin = 0; symbol[begin] != '(' && symbol[begin] != ' '; ++begin);
            for (end = begin; symbol[end] != '+' && symbol[end] != ')' && symbol[end] != ' '; ++end);
            line += std::string(symbol, begin) + "  ";
            std::string functionName = std::string(symbol).substr(begin + 1, end - begin - 1);
            int status;
            char *demangled = abi::__cxa_demangle(functionName.c_str(), 0, 0, &status);
            if (status == 0) {
                line += demangled;
                std::free(demangled);
            } else {
                line += functionName;
            }
            return line;
        }

        static Dl_info symbolInfo(void *addr) {
            Dl_info result;
            int success = dladdr(addr, &result);
            if (not success)
                memset(&result, 0, sizeof(result));
            return result;
        }

        static std::string readTextFile(std::string filename) {
            std::ifstream inputStream(filename);
            std::string result;

            inputStream.seekg(0, std::ios::end);
            result.reserve(inputStream.tellg());
            inputStream.seekg(0, std::ios::beg);

            result.assign((std::istreambuf_iterator<char>(inputStream)),
                          std::istreambuf_iterator<char>());
            return result;
        }

        std::string hash() const {
            unsigned long long result = 0;
            for (unsigned i = 0; i < _traceDepth; i++)
                result ^= reinterpret_cast<unsigned long long>(_trace[i]);
            char text[16];
            sprintf(text, "0x%LX", result);
            return text;
        }

        void *_trace[MAX_BACKTRACE_DEPTH];
        size_t _traceDepth;
    };
}
#endif // __COMMON_BACKTRACE_H__
