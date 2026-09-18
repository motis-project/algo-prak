#include <chrono>
#include <iostream>

#include "fmt/core.h"
#include "fmt/std.h"

#include "conf/options_parser.h"

#include "utl/memory_usage_printer.h"
#include "utl/progress_tracker.h"

#include "osr/cch_preprocessing/preprocess.h"

using namespace osr;
using namespace boost::program_options;

namespace fs = std::filesystem;
namespace prep = cch_preprocessing;
struct config : public conf::configuration {
  config(fs::path dir) : configuration{"Options"}, dir_{std::move(dir)} {
    param(dir_, "in,i", "Routing data directory");
    param(mem_usage_, "mem", "Track memory usage");
  }

  fs::path dir_;
  bool mem_usage_{false};
};

int main(int ac, char const** av) {
  auto opt = config{"./"};
  auto parser = conf::options_parser({&opt});
  parser.read_command_line_args(ac, av);

  if (parser.help()) {
    parser.print_help(std::cout);
    return 0;
  } else if (parser.version()) {
    return 0;
  }

  parser.read_configuration_file();
  parser.print_unrecognized(std::cout);
  parser.print_used(std::cout);

  if (!fs::is_directory(opt.dir_)) {
    fmt::println("directory not found: {}", opt.dir_);
    return 1;
  }

  auto const w = ways{opt.dir_, cista::mmap::protection::READ};

  utl::activate_progress_tracker("osr-cch-preprocess");
  auto const silencer = utl::global_progress_bars{false};

  auto mem = (opt.mem_usage_)
                 ? std::make_unique<utl::memory_usage_printer>(
                       std::cerr, utl::memory_usage_printer::mode::PRINT,
                       std::chrono::seconds{1})
                 : nullptr;

  auto const start = std::chrono::steady_clock::now();

  prep::preprocess(opt.dir_, w);

  if (mem) {
    mem->stop();
    auto const elapsed = std::chrono::steady_clock::now() - start;
    auto const peak = mem->get_peak_memory_usage();

    fmt::println(
        "CCH preprocessing: {:.3f} s, peak RSS: {:.1f} MB, peak virtual: "
        "{:.1f} "
        "MB",
        std::chrono::duration<double>{elapsed}.count(),
        static_cast<double>(peak.rss_) / (1024.0 * 1024.0),
        static_cast<double>(peak.virtual_) / (1024.0 * 1024.0));
  }
}