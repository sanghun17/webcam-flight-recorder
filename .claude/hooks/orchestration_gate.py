#!/usr/bin/env python3
"""orchestration-gate: 메인 에이전트의 코드 파일 수정을 턴당 2개로 제한.

등록: PreToolUse (matcher: Edit|Write|NotebookEdit) + UserPromptSubmit (카운터 리셋).
서브 에이전트(agent_id/agent_type 필드 존재)는 제한 없이 통과.
"""
import json
import os
import sys

CODE_EXT = {
    ".py", ".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".hxx", ".cu", ".cuh",
    ".js", ".ts", ".jsx", ".tsx", ".sh", ".bash", ".zsh", ".go", ".rs",
    ".java", ".kt", ".proto", ".cmake", ".ipynb",
}
LIMIT = 2


def main():
    try:
        data = json.load(sys.stdin)
    except Exception:
        sys.exit(0)  # 입력 파싱 실패 시 열어둔다 (fail-open)

    event = data.get("hook_event_name", "")
    session = data.get("session_id", "unknown")
    state_dir = os.path.join(os.environ.get("TMPDIR") or "/tmp", "claude-orch-gate")
    os.makedirs(state_dir, exist_ok=True)
    state_path = os.path.join(state_dir, f"{session}.json")

    def load_state():
        try:
            with open(state_path) as f:
                return json.load(f)
        except Exception:
            return {"prompt_id": None, "files": []}

    def save_state(s):
        try:
            with open(state_path, "w") as f:
                json.dump(s, f)
        except Exception:
            pass

    # 새 턴 시작: 카운터 리셋
    if event == "UserPromptSubmit":
        save_state({"prompt_id": data.get("prompt_id"), "files": []})
        sys.exit(0)

    # 서브 에이전트의 tool call은 게이트 대상 아님
    if data.get("agent_id") or data.get("agent_type"):
        sys.exit(0)

    tool_input = data.get("tool_input") or {}
    path = tool_input.get("file_path") or tool_input.get("notebook_path") or ""
    if not path:
        sys.exit(0)

    base = os.path.basename(path)
    ext = os.path.splitext(base)[1].lower()
    if ext not in CODE_EXT and base != "CMakeLists.txt":
        sys.exit(0)  # 설정/문서 파일은 제한 없음

    state = load_state()
    prompt_id = data.get("prompt_id")
    # prompt_id가 바뀌었으면 새 턴 (UserPromptSubmit 누락 대비 이중 안전장치)
    if prompt_id is not None and state.get("prompt_id") != prompt_id:
        state = {"prompt_id": prompt_id, "files": []}

    files = state.get("files", [])
    real = os.path.realpath(path)
    if real in files:
        sys.exit(0)  # 이미 카운트된 파일 재수정은 허용

    if len(files) >= LIMIT:
        names = ", ".join(os.path.basename(f) for f in files)
        print(json.dumps({
            "hookSpecificOutput": {
                "hookEventName": "PreToolUse",
                "permissionDecision": "deny",
                "permissionDecisionReason": (
                    f"[orchestration-gate] 이 턴에서 이미 코드 파일 {LIMIT}개를 수정했습니다 ({names}). "
                    "3개 이상의 코드 파일 변경은 메인 에이전트가 직접 하지 말고 서브 에이전트에 위임하세요 "
                    "(.claude/rules/orchestration.md — default-worker/deep-reasoner/task-worker). "
                    "이 제한을 파일 분할 등으로 우회하지 마세요."
                ),
            }
        }))
        sys.exit(0)

    files.append(real)
    state["files"] = files
    if prompt_id is not None:
        state["prompt_id"] = prompt_id
    save_state(state)
    sys.exit(0)


if __name__ == "__main__":
    main()
