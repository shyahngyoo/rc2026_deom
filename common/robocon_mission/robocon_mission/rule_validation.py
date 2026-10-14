"""Read-only scoring extension. Route completion is physical, not a score claim."""
from dataclasses import dataclass
from typing import Optional, Protocol


@dataclass(frozen=True)
class RuleEvidence:
    task_id: str
    skill_completed: bool
    safe_stance: bool
    end_verified: bool
    # Future validator may add timestamped foot contacts and obstacle records.


@dataclass(frozen=True)
class RuleVerdict:
    scored: Optional[bool] = None
    reason: str = "competition rule validator not connected"


class RuleValidator(Protocol):
    def validate(self, evidence: RuleEvidence) -> RuleVerdict:
        ...


class UnconfiguredRuleValidator:
    def validate(self, evidence: RuleEvidence) -> RuleVerdict:
        return RuleVerdict()

