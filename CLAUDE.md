# CLAUDE.md

## Orchestration (필독)

메인 에이전트(Fable)는 지휘만 한다: 계획, 작업 분배, 중요한 결정, 결과 종합.
실제 구현은 서브 에이전트에 위임한다: 무거운 추론은 `deep-reasoner`(Opus), 일반 구현은 `default-worker`(Sonnet), 잡무는 `task-worker`(Haiku).

직접 처리 / 위임 기준, 라우팅, 이 repo 특수 규칙(전역 pkill 금지, :0 사용 금지, 실측 검증):

@.claude/rules/orchestration.md
