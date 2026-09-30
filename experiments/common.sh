# Shared helpers for experiments/expN.sh, source it after the parameter block.
# One run per launch: configure the simulation, start run.sh with logging,
# run the scenario of mors_experiments_sim and stop the whole stack once it finishes.

EXP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${EXP_DIR}/.." && pwd)"
sim_config="${ROOT_DIR}/config/simulation.yaml"
locomotion_config="${ROOT_DIR}/config/locomotion_controller.yaml"

float_re='-?[0-9]+\.[0-9]+'

# require_float NAME...: scenario nodes declare their parameters as doubles,
# an integer literal would be rejected by rclpy.
require_float() {
    local name

    for name in "$@"; do
        if ! [[ "${!name}" =~ ^${float_re}$ ]]; then
            echo "${name}=${!name} must be a float literal (e.g. 1.0)" >&2
            exit 1
        fi
    done
}

# require_float_list NAME...: same for double arrays, e.g. "[0.0, 0.5]"
require_float_list() {
    local name

    for name in "$@"; do
        if ! [[ "${!name}" =~ ^\[\ *${float_re}(\ *,\ *${float_re})*\ *\]$ ]]; then
            echo "${name}=${!name} must be a list of float literals (e.g. [0.0, 0.5])" >&2
            exit 1
        fi
    done
}

# set_yaml_string FILE KEY VALUE: replace a top-level `KEY: "..."` keeping the comment
set_yaml_string() {
    local file="$1" key="$2" value="$3"

    sed -i -E "s/^(${key}:[[:space:]]*)\"[^\"]*\"/\1\"${value}\"/" "$file"
    if ! grep -qE "^${key}:[[:space:]]*\"${value}\"" "$file"; then
        echo "Failed to set ${key} in ${file}" >&2
        exit 1
    fi
}

# "SETTER|FILE|KEY|VALUE" of the overridden keys, restored on exit by restore_configs
config_backup=()

# override_yaml_string FILE KEY VALUE: set_yaml_string remembering the original value
override_yaml_string() {
    local file="$1" key="$2" value="$3" orig

    orig="$(sed -nE "s/^${key}:[[:space:]]*\"([^\"]*)\".*/\1/p" "$file")"
    set_yaml_string "$file" "$key" "$value"
    config_backup+=("set_yaml_string|${file}|${key}|${orig}")
}

# Lists may span several lines; get/set_yaml_list encode the line breaks as a
# literal \n, so a list fits in one line of config_backup and is restored as is.

# get_yaml_list FILE KEY: print a top-level `KEY: [...]`
get_yaml_list() {
    awk -v key="$2" '
        !found && $0 ~ "^" key ":[[:space:]]*\\[" {
            found = 1
            sub("^" key ":[[:space:]]*", "")
            text = ""
        }
        found {
            end = index($0, "]")
            text = text (text == "" ? "" : "\\n") (end ? substr($0, 1, end) : $0)
            if (end) { print text; exit }
        }
    ' "$1"
}

# set_yaml_list FILE KEY VALUE: replace a top-level `KEY: [...]` keeping the comment
set_yaml_list() {
    local file="$1" key="$2" value="$3" text

    text="$(awk -v key="$key" -v value="$value" '
        !done && !inlist && $0 ~ "^" key ":[[:space:]]*\\[" {
            inlist = 1
            match($0, "^" key ":[[:space:]]*")
            prefix = substr($0, 1, RLENGTH)
        }
        inlist {
            end = index($0, "]")
            if (end) {
                print prefix value substr($0, end + 1)
                inlist = 0
                done = 1
            }
            next
        }
        { print }
    ' "$file")"
    printf '%s\n' "$text" > "$file"

    if [ "$(get_yaml_list "$file" "$key")" != "$value" ]; then
        echo "Failed to set ${key} in ${file}" >&2
        exit 1
    fi
}

# override_yaml_list FILE KEY VALUE: set_yaml_list remembering the original value
override_yaml_list() {
    local file="$1" key="$2" value="$3" orig

    orig="$(get_yaml_list "$file" "$key")"
    set_yaml_list "$file" "$key" "$value"
    config_backup+=("set_yaml_list|${file}|${key}|${orig}")
}

# restore_configs: put the overridden keys back in reverse order
restore_configs() {
    local i setter file key value

    for ((i = ${#config_backup[@]} - 1; i >= 0; i--)); do
        IFS='|' read -r setter file key value <<< "${config_backup[i]}"
        # subshell: a failed key must not abort the restore of the others
        ("$setter" "$file" "$key" "$value") || true
    done
    config_backup=()
}

# setup_configs ALGORITHM VEL_CMD_FRAME [SCENE]: scene (flat by default) plus the controller settings
setup_configs() {
    local algorithm="$1" vel_cmd_frame="$2" scene="${3:-flat}"

    case "$algorithm" in
        wbic|vision|dcm) ;;
        *)
            echo "Unknown ALGORITHM: ${algorithm} (supported: wbic, vision, dcm)" >&2
            exit 1
            ;;
    esac

    case "$vel_cmd_frame" in
        local|global) ;;
        *)
            echo "Unknown VEL_CMD_FRAME: ${vel_cmd_frame} (supported: local, global)" >&2
            exit 1
            ;;
    esac

    override_yaml_string "$sim_config" scene "$scene"
    override_yaml_string "$locomotion_config" algorithm "$algorithm"
    override_yaml_string "$locomotion_config" vel_cmd_frame "$vel_cmd_frame"

    config_params=(algorithm="$algorithm" vel_cmd_frame="$vel_cmd_frame" scene="$scene")
}

# setup_kin_scheme KIN_SCHEME: initial leg configuration -> init_motor_angles,
# call after setup_configs
setup_kin_scheme() {
    local kin_scheme="$1" init_motor_angles

    case "$kin_scheme" in
        ">>") init_motor_angles="[0.0, 1.57, -3.14, -0.0, -1.57, 3.14, -0.0, 1.57, -3.14, 0.0, -1.57, 3.14]" ;;
        "<<") init_motor_angles="[0.0, -1.57, 3.14, -0.0, 1.57, -3.14, -0.0, -1.57, 3.14, 0.0, 1.57, -3.14]" ;;
        "><") init_motor_angles="[0.0, -1.57, 3.14, -0.0, 1.57, -3.14, -0.0, 1.57, -3.14, 0.0, -1.57, 3.14]" ;;
        "<>") init_motor_angles="[0.0, 1.57, -3.14, -0.0, -1.57, 3.14, -0.0, -1.57, 3.14, 0.0, 1.57, -3.14]" ;;
        *)
            echo "Unknown KIN_SCHEME: ${kin_scheme} (supported: >>, <<, ><, <>)" >&2
            exit 1
            ;;
    esac

    override_yaml_list "$sim_config" init_motor_angles "$init_motor_angles"
    config_params+=(kin_scheme="$kin_scheme" init_motor_angles="$init_motor_angles")
}

run_pid=""

cleanup() {
    local status=$?

    trap - EXIT
    if [ -n "$run_pid" ] && kill -0 "$run_pid" 2>/dev/null; then
        # SIGINT is ignored by background jobs of a non-interactive shell,
        # SIGTERM triggers the graceful cleanup of run.sh.
        kill -TERM "$run_pid" 2>/dev/null || true
        wait "$run_pid" 2>/dev/null || true
    fi
    restore_configs
    exit "$status"
}

# Installed on source, so the configs are restored on any exit, including
# a failure between setup_configs and run_experiment.
trap 'exit 130' SIGINT
trap 'exit 143' SIGTERM
trap cleanup EXIT

# run_experiment EXP RUN_NAME MAX_DURATION ROS_PARAM...
#   EXP           executable of mors_experiments_sim, also the log folder prefix
#   RUN_NAME      log subfolder: experiments/<EXP>_logs/<RUN_NAME>
#   MAX_DURATION  upper bound of the scenario duration [s]
#   ROS_PARAM     name:=value
# SCENARIO (optional) overrides the executable, e.g. SCENARIO=exp1 run_experiment exp3 ...
run_experiment() {
    local exp="$1" run_name="$2" max_duration="$3" scenario="${SCENARIO:-$1}"
    shift 3

    local log_dir="${EXP_DIR}/${exp}_logs/${run_name}"
    local param ros_args=()

    for param in "$@"; do
        ros_args+=(-p "$param")
    done

    # MorsLogger creates log_<datetime>/ inside, so repeated runs do not collide.
    mkdir -p "$log_dir"
    {
        echo "date=$(date '+%Y-%m-%d %H:%M:%S')"
        printf '%s\n' "${config_params[@]}"
        for param in "$@"; do
            echo "${param/:=/=}"
        done
    } > "${log_dir}/params.txt"

    # run.sh stops the logger after 120 s by default.
    local run_args=(--sim --rviz --log-dir "$log_dir")
    local log_time
    log_time="$(awk -v d="$STARTUP_DELAY" -v t="$max_duration" \
        'BEGIN { t = d + t + 10.0; if (t > 120) printf "%d", t + 1 }')"
    if [ -n "$log_time" ]; then
        run_args+=(--log-time "$log_time")
    fi

    echo "[${exp}]: logs -> ${log_dir}"
    "${ROOT_DIR}/run.sh" "${run_args[@]}" &
    run_pid=$!

    sleep "$STARTUP_DELAY"
    if ! kill -0 "$run_pid" 2>/dev/null; then
        echo "[${exp}]: run.sh exited during startup" >&2
        exit 1
    fi

    ros2 run mors_experiments_sim "$scenario" --ros-args "${ros_args[@]}"

    echo "[${exp}]: finished"
}
