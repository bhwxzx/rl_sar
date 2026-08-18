#include "lw_deployment_bundle.hpp"

#include <filesystem>
#include <fstream>
#include <functional>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

namespace
{
namespace fs = std::filesystem;

const std::string kCommit(40, 'a');
const std::vector<std::string> kPolicyFiles = {
    "policy/LW/base.yaml",
    "policy/LW/robot_lab/leg_loco/config.yaml",
    "policy/LW/robot_lab/leg_loco/policy.onnx",
    "policy/LW/robot_lab/leg_to_wheel/config.yaml",
    "policy/LW/robot_lab/leg_to_wheel/leg_to_wheel_transform_60hz.csv",
    "policy/LW/robot_lab/leg_to_wheel/policy.onnx",
    "policy/LW/robot_lab/wheel_loco/config.yaml",
    "policy/LW/robot_lab/wheel_loco/policy.onnx",
    "policy/LW/robot_lab/wheel_to_leg/config.yaml",
    "policy/LW/robot_lab/wheel_to_leg/policy.onnx",
    "policy/LW/robot_lab/wheel_to_leg/wheel_to_leg_transform_60hz.csv",
};
const std::vector<std::string> kRuntimeFiles = {
    "lib/libserial.a",
    "lib/fdilink_ahrs/ahrs_driver_node",
    "lib/rl_sar/lw_config_profiler",
    "lib/rl_sar/profile_lw_runtime_config.py",
    "share/ament_index/resource_index/packages/fdilink_ahrs",
    "share/ament_index/resource_index/packages/rl_sar",
    "share/ament_index/resource_index/packages/serial",
    "share/fdilink_ahrs/launch/ahrs_driver.launch.py",
    "share/fdilink_ahrs/package.xml",
    "share/fdilink_ahrs/wheeltec_udev.sh",
    "share/rl_sar/launch/rl_real_LW.launch.py",
    "share/serial/package.xml",
};
const std::vector<std::string> kOnnxRuntimeFiles = {
    "lib/rl_sar/onnxruntime/libonnxruntime.so.1",
    "lib/rl_sar/onnxruntime/libonnxruntime_providers_shared.so",
};
const std::string kOnnxOrigin = "lib/rl_sar/onnxruntime/origin.json";

#if defined(__x86_64__) || defined(_M_X64)
const std::string kArchitecture = "x86_64";
const std::string kArchiveName = "onnxruntime-linux-x64-1.22.0.tgz";
const std::string kArchiveUrl =
    "https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz";
const std::string kArchiveSha256 =
    "8344d55f93d5bc5021ce342db50f62079daf39aaafb5d311a451846228be49b3";
constexpr std::uint16_t kElfMachine = 62;
constexpr std::uint16_t kOtherElfMachine = 183;
#elif defined(__aarch64__) || defined(_M_ARM64)
const std::string kArchitecture = "aarch64";
const std::string kArchiveName = "onnxruntime-linux-aarch64-1.22.0.tgz";
const std::string kArchiveUrl =
    "https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz";
const std::string kArchiveSha256 =
    "bb76395092d150b52c7092dc6b8f2fe4d80f0f3bf0416d2f269193e347e24702";
constexpr std::uint16_t kElfMachine = 183;
constexpr std::uint16_t kOtherElfMachine = 62;
#else
#error Unsupported test architecture
#endif

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void writeFile(const fs::path& path, const std::string& content)
{
    fs::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::binary);
    if (!output)
    {
        throw std::runtime_error("failed to create " + path.string());
    }
    output << content;
}

std::string elf64Bytes(std::uint16_t machine)
{
    std::string header(20, '\0');
    header[0] = 0x7f;
    header[1] = 'E';
    header[2] = 'L';
    header[3] = 'F';
    header[4] = 2;
    header[5] = 1;
    header[6] = 1;
    header[18] = static_cast<char>(machine & 0xff);
    header[19] = static_cast<char>((machine >> 8) & 0xff);
    return header + "test ELF payload\n";
}

class Fixture
{
public:
    Fixture()
    {
        std::string pattern = "/tmp/lw-deployment-test-XXXXXX";
        std::vector<char> storage(pattern.begin(), pattern.end());
        storage.push_back('\0');
        char* created = mkdtemp(storage.data());
        if (!created)
        {
            throw std::runtime_error("mkdtemp failed");
        }
        root = created;
        prefix = root / "install";
        share = prefix / "share" / "rl_sar";
        bundle = share / "deployment" / "LW";
        executable = prefix / "lib" / "rl_sar" / "rl_real_LW";

        writeFile(executable, "test executable bytes\n");
        for (const std::string& relative : kPolicyFiles)
        {
            writeFile(bundle / relative, "asset:" + relative + "\n");
        }
        for (const std::string& relative : kRuntimeFiles)
        {
            writeFile(prefix / relative, "runtime:" + relative + "\n");
        }
        for (const std::string& relative : kOnnxRuntimeFiles)
        {
            const std::string content = relative == kOnnxRuntimeFiles.back()
                ? elf64Bytes(kElfMachine) + "provider\n"
                : elf64Bytes(kElfMachine);
            writeFile(prefix / relative, content);
        }
        writeFile(prefix / kOnnxOrigin, "{\"approved\":true}\n");
        writeManifest();
    }

    ~Fixture()
    {
        std::error_code error;
        fs::remove_all(root, error);
    }

    void writeManifest(
        const std::string& source_commit = kCommit,
        const std::string& first_manifest_path = "",
        const std::string& onnx_version = "1.22.0",
        const std::string& onnx_architecture = kArchitecture,
        const std::string& archive_sha256 = kArchiveSha256)
    {
        std::ostringstream manifest;
        manifest << "schema_version: 4\n"
                 << "source_commit: \"" << source_commit << "\"\n"
                 << "build_type: \"Release\"\n"
                 << "executable:\n"
                 << "  name: \"rl_real_LW\"\n"
                 << "  sha256: \""
                 << LWDeploymentBundle::Sha256File(executable)
                 << "\"\n"
                 << "onnx_runtime:\n"
                 << "  version: \"" << onnx_version << "\"\n"
                 << "  architecture: \"" << onnx_architecture << "\"\n"
                 << "  archive:\n"
                 << "    name: \"" << kArchiveName << "\"\n"
                 << "    url: \"" << kArchiveUrl << "\"\n"
                 << "    sha256: \"" << archive_sha256 << "\"\n"
                 << "  provenance:\n"
                 << "    path: \"" << kOnnxOrigin << "\"\n"
                 << "    sha256: \""
                 << LWDeploymentBundle::Sha256File(prefix / kOnnxOrigin)
                 << "\"\n"
                 << "  libraries:\n";
        for (const std::string& relative : kOnnxRuntimeFiles)
        {
            manifest << "    - path: \"" << relative << "\"\n"
                     << "      sha256: \""
                     << LWDeploymentBundle::Sha256File(prefix / relative)
                     << "\"\n";
        }
        manifest << "files:\n";
        for (std::size_t index = 0; index < kPolicyFiles.size(); ++index)
        {
            const std::string& actual_path = kPolicyFiles[index];
            const std::string manifest_path =
                index == 0 && !first_manifest_path.empty()
                ? first_manifest_path
                : actual_path;
            manifest << "  - path: \"" << manifest_path << "\"\n"
                     << "    sha256: \""
                     << LWDeploymentBundle::Sha256File(bundle / actual_path)
                     << "\"\n";
        }
        manifest << "runtime_files:\n";
        for (const std::string& relative : kRuntimeFiles)
        {
            manifest << "  - path: \"" << relative << "\"\n"
                     << "    sha256: \""
                     << LWDeploymentBundle::Sha256File(prefix / relative)
                     << "\"\n";
        }
        writeFile(bundle / "manifest.yaml", manifest.str());
    }

    fs::path root;
    fs::path prefix;
    fs::path share;
    fs::path bundle;
    fs::path executable;
};

void requireFailure(
    const std::function<void()>& operation,
    const std::string& expected_text)
{
    try
    {
        operation();
    }
    catch (const std::exception& exception)
    {
        const std::string message = exception.what();
        require(
            message.find(expected_text) != std::string::npos,
            "unexpected validation error: " + message);
        return;
    }
    throw std::runtime_error(
        "expected validation failure containing: " + expected_text);
}

void testValidAndRelocatableBundle()
{
    Fixture fixture;
    const LWDeploymentBundleInfo initial = LWDeploymentBundle::Verify(
        fixture.share, fixture.executable, kCommit);
    require(initial.files.size() == kPolicyFiles.size(), "wrong file count");
    require(initial.runtime_files.size() == kRuntimeFiles.size(),
            "wrong runtime file count");
    require(initial.onnx_runtime_libraries.size() == kOnnxRuntimeFiles.size(),
            "wrong ONNX Runtime library count");
    require(initial.onnx_runtime_version == "1.22.0",
            "wrong ONNX Runtime version");
    require(initial.onnx_runtime_architecture == kArchitecture,
            "wrong ONNX Runtime architecture");
    require(initial.onnx_runtime_archive_name == kArchiveName,
            "wrong ONNX Runtime archive name");
    require(initial.onnx_runtime_archive_url == kArchiveUrl,
            "wrong ONNX Runtime archive URL");
    require(initial.onnx_runtime_archive_sha256 == kArchiveSha256,
            "wrong ONNX Runtime archive SHA-256");
    require(initial.source_commit == kCommit, "wrong source commit");

    const fs::path moved_prefix = fixture.root / "moved-install";
    fs::rename(fixture.prefix, moved_prefix);
    fixture.prefix = moved_prefix;
    fixture.share = moved_prefix / "share" / "rl_sar";
    fixture.bundle = fixture.share / "deployment" / "LW";
    fixture.executable = moved_prefix / "lib" / "rl_sar" / "rl_real_LW";
    const LWDeploymentBundleInfo moved = LWDeploymentBundle::Verify(
        fixture.share, fixture.executable, kCommit);
    require(moved.package_prefix == fs::canonical(moved_prefix),
            "moved prefix was not resolved correctly");
}

void testMissingAssetFails()
{
    Fixture fixture;
    fs::remove(fixture.bundle / kPolicyFiles.front());
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "regular non-symlink file");
}

void testTamperedAssetFails()
{
    Fixture fixture;
    std::ofstream output(
        fixture.bundle / kPolicyFiles.back(),
        std::ios::binary | std::ios::app);
    output << "tampered";
    output.close();
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "SHA-256 mismatch");
}

void testTamperedExecutableFails()
{
    Fixture fixture;
    std::ofstream output(
        fixture.executable,
        std::ios::binary | std::ios::app);
    output << "tampered";
    output.close();
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "executable SHA-256 mismatch");
}

void testMissingRuntimeDependencyFails()
{
    Fixture fixture;
    fs::remove(fixture.prefix / kRuntimeFiles.front());
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "runtime dependency must be a regular non-symlink file");
}

void testTamperedRuntimeDependencyFails()
{
    Fixture fixture;
    std::ofstream output(
        fixture.prefix / kRuntimeFiles.back(),
        std::ios::binary | std::ios::app);
    output << "tampered";
    output.close();
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "runtime SHA-256 mismatch");
}

void testSymlinkRuntimeDependencyFails()
{
    Fixture fixture;
    const fs::path runtime = fixture.prefix / kRuntimeFiles.front();
    const fs::path external = fixture.root / "external-runtime";
    writeFile(external, "external\n");
    fs::remove(runtime);
    fs::create_symlink(external, runtime);
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "symbolic link");
}

void testMissingProductionLaunchFails()
{
    Fixture fixture;
    fs::remove(
        fixture.prefix / "share/rl_sar/launch/rl_real_LW.launch.py");
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "runtime dependency must be a regular non-symlink file");
}

void testTamperedProductionLaunchFails()
{
    Fixture fixture;
    std::ofstream output(
        fixture.prefix / "share/rl_sar/launch/rl_real_LW.launch.py",
        std::ios::binary | std::ios::app);
    output << "tampered";
    output.close();
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "runtime SHA-256 mismatch");
}

void testSymlinkProductionLaunchFails()
{
    Fixture fixture;
    const fs::path launch =
        fixture.prefix / "share/rl_sar/launch/rl_real_LW.launch.py";
    const fs::path external = fixture.root / "external-launch.py";
    writeFile(external, "external launch\n");
    fs::remove(launch);
    fs::create_symlink(external, launch);
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "symbolic link");
}

void testSymlinkProductionLaunchDirectoryFails()
{
    Fixture fixture;
    const fs::path launch_directory =
        fixture.prefix / "share/rl_sar/launch";
    const fs::path external = fixture.root / "external-launch-directory";
    fs::rename(launch_directory, external);
    fs::create_directory_symlink(external, launch_directory);
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "symbolic link");
}

void testExtraProductionLaunchFails()
{
    Fixture fixture;
    writeFile(
        fixture.prefix / "share/rl_sar/launch/unapproved.launch.py",
        "development launch\n");
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "exact approved file set");
}

void testProductionLaunchBytecodeCacheFails()
{
    Fixture fixture;
    writeFile(
        fixture.prefix
            / "share/rl_sar/launch/__pycache__/rl_real_LW.launch.pyc",
        "unapproved bytecode\n");
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "non-regular entry");
}

void testMissingOnnxRuntimeLibraryFails()
{
    Fixture fixture;
    fs::remove(fixture.prefix / kOnnxRuntimeFiles.front());
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "ONNX Runtime library must be a regular non-symlink file");
}

void testTamperedOnnxRuntimeLibraryFails()
{
    Fixture fixture;
    std::ofstream output(
        fixture.prefix / kOnnxRuntimeFiles.back(),
        std::ios::binary | std::ios::app);
    output << "tampered";
    output.close();
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "ONNX Runtime SHA-256 mismatch");
}

void testSelfConsistentUnapprovedOnnxRuntimeLibraryFails()
{
    Fixture fixture;
    std::ofstream output(
        fixture.prefix / kOnnxRuntimeFiles.front(),
        std::ios::binary | std::ios::app);
    output << "tampered";
    output.close();
    fixture.writeManifest();
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "library SHA-256 is not approved");
}

void testTamperedOnnxRuntimeProvenanceFails()
{
    Fixture fixture;
    std::ofstream output(
        fixture.prefix / kOnnxOrigin,
        std::ios::binary | std::ios::app);
    output << "tampered";
    output.close();
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "provenance SHA-256 mismatch");
}

void testSymlinkOnnxRuntimeLibraryFails()
{
    Fixture fixture;
    const fs::path library = fixture.prefix / kOnnxRuntimeFiles.front();
    const fs::path external = fixture.root / "external-onnx";
    writeFile(external, elf64Bytes(kElfMachine));
    fs::remove(library);
    fs::create_symlink(external, library);
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "symbolic link");
}

void testWrongOnnxRuntimeElfArchitectureFails()
{
    Fixture fixture;
    writeFile(
        fixture.prefix / kOnnxRuntimeFiles.back(),
        elf64Bytes(kOtherElfMachine));
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "ONNX Runtime architecture mismatch");
}

void testManifestOnnxRuntimeArchitectureMismatchFails()
{
    Fixture fixture;
    const std::string other_architecture =
        kArchitecture == "x86_64" ? "aarch64" : "x86_64";
    fixture.writeManifest(kCommit, "", "1.22.0", other_architecture);
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "architecture does not match the binary");
}

void testInvalidOnnxRuntimeVersionFails()
{
    Fixture fixture;
    fixture.writeManifest(kCommit, "", "latest");
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "ONNX Runtime version is invalid");
}

void testUnapprovedOnnxRuntimeVersionAndArchiveFail()
{
    Fixture fixture;
    fixture.writeManifest(kCommit, "", "1.23.0");
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "version is not approved");

    fixture.writeManifest(kCommit, "", "1.22.0", kArchitecture, std::string(64, 'c'));
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "archive provenance is not approved");
}

void testExtraOnnxRuntimeLibraryFails()
{
    Fixture fixture;
    writeFile(
        fixture.prefix / "lib/rl_sar/onnxruntime/libunexpected.so",
        elf64Bytes(kElfMachine));
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "exact approved library set");
}

void testSourceCommitMismatchFails()
{
    Fixture fixture;
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share,
                fixture.executable,
                std::string(40, 'b'));
        },
        "source_commit does not match");
}

void testSymlinkAssetFails()
{
    Fixture fixture;
    const fs::path asset = fixture.bundle / kPolicyFiles.front();
    const fs::path external = fixture.root / "external.yaml";
    writeFile(external, "external\n");
    fs::remove(asset);
    fs::create_symlink(external, asset);
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "symbolic link");
}

void testSymlinkInstalledExecutableFails()
{
    Fixture fixture;
    const fs::path external = fixture.root / "external-rl_real_LW";
    fs::rename(fixture.executable, external);
    fs::create_symlink(external, fixture.executable);
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, external, kCommit);
        },
        "installed executable must be a regular non-symlink file");
}

void testNonNormalizedManifestPathFails()
{
    Fixture fixture;
    fixture.writeManifest(
        kCommit,
        "policy/LW/robot_lab/../base.yaml");
    requireFailure(
        [&]() {
            LWDeploymentBundle::Verify(
                fixture.share, fixture.executable, kCommit);
        },
        "not normalized");
}
}

int main()
{
    try
    {
        testValidAndRelocatableBundle();
        testMissingAssetFails();
        testTamperedAssetFails();
        testTamperedExecutableFails();
        testMissingRuntimeDependencyFails();
        testTamperedRuntimeDependencyFails();
        testSymlinkRuntimeDependencyFails();
        testMissingProductionLaunchFails();
        testTamperedProductionLaunchFails();
        testSymlinkProductionLaunchFails();
        testSymlinkProductionLaunchDirectoryFails();
        testExtraProductionLaunchFails();
        testProductionLaunchBytecodeCacheFails();
        testMissingOnnxRuntimeLibraryFails();
        testTamperedOnnxRuntimeLibraryFails();
        testSelfConsistentUnapprovedOnnxRuntimeLibraryFails();
        testTamperedOnnxRuntimeProvenanceFails();
        testSymlinkOnnxRuntimeLibraryFails();
        testWrongOnnxRuntimeElfArchitectureFails();
        testManifestOnnxRuntimeArchitectureMismatchFails();
        testInvalidOnnxRuntimeVersionFails();
        testUnapprovedOnnxRuntimeVersionAndArchiveFail();
        testExtraOnnxRuntimeLibraryFails();
        testSourceCommitMismatchFails();
        testSymlinkAssetFails();
        testSymlinkInstalledExecutableFails();
        testNonNormalizedManifestPathFails();
    }
    catch (const std::exception& exception)
    {
        std::cerr << "test_lw_deployment_bundle failed: "
                  << exception.what() << std::endl;
        return 1;
    }
    std::cout << "test_lw_deployment_bundle passed" << std::endl;
    return 0;
}
