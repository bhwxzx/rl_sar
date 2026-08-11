#include "lw_deployment_bundle.hpp"

#include <filesystem>
#include <fstream>
#include <functional>
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
    "share/ament_index/resource_index/packages/serial",
    "share/fdilink_ahrs/launch/ahrs_driver.launch.py",
    "share/fdilink_ahrs/package.xml",
    "share/fdilink_ahrs/wheeltec_udev.sh",
    "share/serial/package.xml",
};

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
        writeManifest();
    }

    ~Fixture()
    {
        std::error_code error;
        fs::remove_all(root, error);
    }

    void writeManifest(
        const std::string& source_commit = kCommit,
        const std::string& first_manifest_path = "")
    {
        std::ostringstream manifest;
        manifest << "schema_version: 2\n"
                 << "source_commit: \"" << source_commit << "\"\n"
                 << "build_type: \"Release\"\n"
                 << "executable:\n"
                 << "  name: \"rl_real_LW\"\n"
                 << "  sha256: \""
                 << LWDeploymentBundle::Sha256File(executable)
                 << "\"\n"
                 << "files:\n";
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
