#!/usr/bin/env bash
# DEPRECATED NAME — kept as a compatibility shim (2026-09-03).
#
# Renamed to run_landing.sh. The "aruco" in the old name was misleading: WORLD and
# MARKER_TYPE are env-driven, so this launcher flies the cross marker whenever it is told
# to — which it is, by the standing rule (WORLD=cross_marker MARKER_TYPE=cross). The name
# caused a real "are you running ArUco?" doubt during a cross-marker-only session.
#
# 184 references across code, docs, memory and ~50 committed A/B harnesses point at the old
# name, so it stays as this shim rather than being deleted. USE run_landing.sh IN NEW WORK.
exec "$(dirname "${BASH_SOURCE[0]}")/run_landing.sh" "$@"
