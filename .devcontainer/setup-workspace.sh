#!/usr/bin/env bash
#
# Bootstrap the ROS 2 workspace of the UR5e + VLA stack.
#
#   1. fix the ownership of the Docker volumes (they are created as root),
#   2. import the pinned third-party repositories (ur5e_vla.repos),
#   3. build the workspace (only the dependency closure we actually use).
#
# Idempotent: each step is skipped when its marker is up to date, so the script
# can be re-run after every container recreation. Run it from anywhere:
#
#     bash /home/ws/src/ur5e_vla_ros2/.devcontainer/setup-workspace.sh
#
set -eo pipefail
# NOTE: no `set -u` -- the ROS setup scripts reference variables that are not
# defined in a non-interactive shell (e.g. AMENT_TRACE_SETUP_FILES).

REPO_DIR="${REPO_DIR:-/home/ws/src/ur5e_vla_ros2}"
WS_ROOT="${WS_ROOT:-/home/ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REPOS_FILE="${REPOS_FILE:-${REPO_DIR}/ur5e_vla.repos}"
IMPORT_MARKER="${WS_ROOT}/src/.vcs_imported"
BUILD_MARKER="${WS_ROOT}/install/.colcon_built"

log() { printf '\n[setup-workspace] %s\n' "$*"; }
fail() { printf '\n[setup-workspace] ERROR: %s\n' "$*" >&2; exit 1; }

# --- 0. sanity -------------------------------------------------------------
[ -f "${REPOS_FILE}" ] || fail "missing dependency file: ${REPOS_FILE}"
command -v vcs >/dev/null 2>&1 || fail \
    "vcs (vcstool) is not installed - add python3-vcstool to devcontainer.containerfile"

# --- 1. ownership ----------------------------------------------------------
# The named volumes are created as root; take them over so that colcon and vcs
# work as the regular user without sudo.
log "Fixing ownership of the workspace volumes"
sudo chown -R "$(whoami):$(whoami)" \
    "${WS_ROOT}/build" "${WS_ROOT}/install" "${WS_ROOT}/log" "${WS_ROOT}/src" \
    2>/dev/null || true
sudo install -d -o "$(whoami)" -g "$(whoami)" \
    /home/ros2dev/.config/opencode \
    /home/ros2dev/.local/share/opencode \
    /home/ros2dev/.cache/opencode \
    /home/ros2dev/.local/state \
    /tmp/opencode

# --- 2. import the pinned dependencies -------------------------------------
REPOS_HASH="$(sha256sum "${REPOS_FILE}" | cut -d' ' -f1)"
if [ -f "${IMPORT_MARKER}" ] && [ "$(cat "${IMPORT_MARKER}")" = "${REPOS_HASH}" ]; then
    log "Dependencies already imported (${REPOS_HASH:0:12}) - skipping"
else
    log "Importing pinned dependencies from ${REPOS_FILE}"
    vcs import "${WS_ROOT}/src" < "${REPOS_FILE}"
    printf '%s' "${REPOS_HASH}" > "${IMPORT_MARKER}"
fi

# --- 3. system dependencies of the imported packages ----------------------
# The packages we build from source need system libraries that the ROS base
# image does not necessarily carry (e.g. realtime_tools needs libcap-dev for
# sys/capability.h). rosdep resolves them from the package manifests, so the
# list does not have to be maintained by hand.
log "Resolving system dependencies with rosdep"
sudo apt-get update -qq || true
# Register our local rules (ament_python & friends are missing from the
# rosdep index in this environment).
sudo sh -c "echo 'yaml file://${REPO_DIR}/.devcontainer/rosdep-local.yaml' \
    > /etc/ros/rosdep/sources.list.d/10-local.list" || true
rosdep update --rosdistro "${ROS_DISTRO}" >/dev/null 2>&1 || true
rosdep install --from-paths "${WS_ROOT}/src" --ignore-src --rosdistro "${ROS_DISTRO}" -y \
    || fail "rosdep could not install the system dependencies"

# --- 4. build --------------------------------------------------------------
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Build only what we need: our own packages when they exist, otherwise the
# upstream stack they depend on. --packages-up-to resolves the dependency
# closure, so unused packages of the imported monorepos are not built.
PACKAGES=()
for pkg in ur5e_vla_description ur5e_vla_control ur5e_vla_bringup; do
    [ -d "${REPO_DIR}/src/${pkg}" ] && PACKAGES+=("${pkg}")
done

if [ ${#PACKAGES[@]} -eq 0 ]; then
    log "Own packages not present yet - building the upstream stack"
    PACKAGES=(
        robotiq_driver robotiq_controllers robotiq_description
        ur_robot_driver ur_description ur_calibration ur_controllers
        parallel_gripper_controller joint_state_broadcaster
        usb_cam
    )
fi

if [ -f "${BUILD_MARKER}" ] && [ "$(cat "${BUILD_MARKER}")" = "${REPOS_HASH}" ]; then
    log "Workspace already built for this dependency set - skipping"
else
    log "Building: ${PACKAGES[*]}"
    log "(this takes a while: ros2_control, the UR driver and the controllers compile from source)"
    # BUILD_TESTING=OFF for the third-party packages: their unit tests pull in a
    # chain of test-only interface packages (test_msgs -> test_interface_files ->
    # rosidl_cmake) that would have to be rebuilt from source as well and would
    # clash with the apt-provided rosidl stack. Our own tests are pytest-based
    # (ament_python) and run regardless of this flag.
    ( cd "${WS_ROOT}" && colcon build --symlink-install \
        --packages-up-to "${PACKAGES[@]}" \
        --cmake-args -DBUILD_TESTING=OFF )
    printf '%s' "${REPOS_HASH}" > "${BUILD_MARKER}"
fi

# --- 5. done ---------------------------------------------------------------
sudo chown -R "$(whoami):$(whoami)" "${WS_ROOT}/build" "${WS_ROOT}/install" \
    "${WS_ROOT}/log" 2>/dev/null || true

log "Workspace ready. Activate it with:"
log "    source ${WS_ROOT}/install/setup.bash"
