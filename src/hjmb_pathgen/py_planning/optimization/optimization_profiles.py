"""Central Phase 6 optimization profile budgets."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

from hjmb_pathgen.py_domain.leg_optimization import LegOptimizationProfileName


@dataclass(frozen=True)
class OptimizationProfile:
    name: LegOptimizationProfileName
    max_spacing_mm: float
    oversample_per_segment: int
    max_initial_guesses: int
    coordinate_passes: int
    coordinate_step_mm: float
    time_budget_ms: int
    strict_collision: bool
    yaw_alpha_values: tuple[float, ...]
    yaw_window_pairs: tuple[tuple[float, float], ...]
    random_variant_count: int = 0


DEFAULT_PROFILES = {
    LegOptimizationProfileName.QUICK_PREVIEW: OptimizationProfile(
        name=LegOptimizationProfileName.QUICK_PREVIEW,
        max_spacing_mm=50.0,
        oversample_per_segment=24,
        max_initial_guesses=3,
        coordinate_passes=0,
        coordinate_step_mm=80.0,
        time_budget_ms=1500,
        strict_collision=False,
        yaw_alpha_values=(0.5,),
        yaw_window_pairs=((0.25, 0.75),),
        random_variant_count=0,
    ),
    LegOptimizationProfileName.AUTOMATIC: OptimizationProfile(
        name=LegOptimizationProfileName.AUTOMATIC,
        max_spacing_mm=25.0,
        oversample_per_segment=48,
        max_initial_guesses=4,
        coordinate_passes=1,
        coordinate_step_mm=55.0,
        time_budget_ms=30000,
        strict_collision=True,
        # Keep the automatic pass bounded: geometry candidates and coordinate
        # refinement are more valuable than multiplying every expensive strict
        # collision check by nine yaw schedules.  The selected XY path still
        # receives a balanced yaw window; FINAL can perform the broader search.
        yaw_alpha_values=(0.50,),
        yaw_window_pairs=((0.25, 0.75),),
        random_variant_count=0,
    ),
    LegOptimizationProfileName.STANDARD: OptimizationProfile(
        name=LegOptimizationProfileName.STANDARD,
        max_spacing_mm=25.0,
        oversample_per_segment=48,
        max_initial_guesses=6,
        coordinate_passes=1,
        coordinate_step_mm=60.0,
        time_budget_ms=30000,
        strict_collision=True,
        yaw_alpha_values=(0.25, 0.5, 0.75),
        yaw_window_pairs=((0.15, 0.65), (0.25, 0.75), (0.35, 0.85)),
        random_variant_count=2,
    ),
    LegOptimizationProfileName.FINAL: OptimizationProfile(
        name=LegOptimizationProfileName.FINAL,
        max_spacing_mm=15.0,
        oversample_per_segment=80,
        max_initial_guesses=8,
        coordinate_passes=2,
        coordinate_step_mm=40.0,
        time_budget_ms=300000,
        strict_collision=True,
        yaw_alpha_values=(0.2, 0.35, 0.5, 0.65, 0.8),
        yaw_window_pairs=((0.10, 0.60), (0.20, 0.70), (0.30, 0.80), (0.40, 0.90)),
        random_variant_count=4,
    ),
}


def optimization_profile_from_project(name: LegOptimizationProfileName, project_profiles: dict[str, Any], *, override_time_budget_ms: int | None = None) -> OptimizationProfile:
    profile = DEFAULT_PROFILES[name]
    raw = project_profiles.get(name.value, {}) if isinstance(project_profiles, dict) else {}
    if isinstance(raw, dict):
        profile = replace(
            profile,
            max_spacing_mm=float(raw.get("max_spacing_mm", profile.max_spacing_mm)),
            oversample_per_segment=int(raw.get("oversample_per_segment", profile.oversample_per_segment)),
            max_initial_guesses=int(raw.get("max_initial_guesses", profile.max_initial_guesses)),
            coordinate_passes=int(raw.get("coordinate_passes", profile.coordinate_passes)),
            coordinate_step_mm=float(raw.get("coordinate_step_mm", profile.coordinate_step_mm)),
            time_budget_ms=int(raw.get("time_budget_ms", profile.time_budget_ms)),
            strict_collision=bool(raw.get("strict_collision", profile.strict_collision)),
            yaw_alpha_values=tuple(float(value) for value in raw.get("yaw_alpha_values", profile.yaw_alpha_values)),
            yaw_window_pairs=tuple(tuple(float(item) for item in pair) for pair in raw.get("yaw_window_pairs", profile.yaw_window_pairs)),
            random_variant_count=int(raw.get("random_variant_count", profile.random_variant_count)),
        )
    if override_time_budget_ms is not None:
        profile = replace(profile, time_budget_ms=int(override_time_budget_ms))
    return _validated_profile(profile)


def _validated_profile(profile: OptimizationProfile) -> OptimizationProfile:
    if profile.max_spacing_mm <= 0:
        raise ValueError("max_spacing_mm must be positive")
    if profile.oversample_per_segment < 8:
        raise ValueError("oversample_per_segment must be at least 8")
    if profile.max_initial_guesses <= 0:
        raise ValueError("max_initial_guesses must be positive")
    if profile.coordinate_passes < 0:
        raise ValueError("coordinate_passes must be non-negative")
    if profile.coordinate_step_mm < 0:
        raise ValueError("coordinate_step_mm must be non-negative")
    if profile.time_budget_ms <= 0:
        raise ValueError("time_budget_ms must be positive")
    if profile.random_variant_count < 0:
        raise ValueError("random_variant_count must be non-negative")
    if not profile.yaw_alpha_values:
        raise ValueError("at least one yaw alpha candidate is required")
    for value in profile.yaw_alpha_values:
        if not 0.0 <= value <= 1.0:
            raise ValueError("yaw alpha candidates must be in [0, 1]")
    if not profile.yaw_window_pairs:
        raise ValueError("at least one yaw window candidate is required")
    for start_end, finish_start in profile.yaw_window_pairs:
        if not 0.0 <= start_end <= finish_start <= 1.0:
            raise ValueError("yaw window candidates must satisfy 0 <= start_end <= finish_start <= 1")
    return profile
