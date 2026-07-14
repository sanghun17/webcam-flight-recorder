#!/usr/bin/env bash
# orchestration-gate: 메인 에이전트의 코드 파일 수정을 턴당 2개로 제한.
# 로직은 orchestration_gate.py (stdin의 hook JSON을 그대로 넘긴다).
exec python3 "$(dirname "$0")/orchestration_gate.py"
