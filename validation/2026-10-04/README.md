# October 4 validation

The flight source is the installed October 3 candidate. The only later mission
repository change is analysis/reporting of failed failsafe touchdown samples;
no flight-control code changed after the reproduced flight.

- 133 flight regression tests, deterministic generation/compilation, and Ruff pass.
- Five connector subprocess regression tests pass on Windows and Linux.
- The selected ten controller helper modules have 87% statement/branch combined
  coverage. This is targeted coverage, not whole-controller or flight readiness.
- Fresh gusts plus 12-second GNSS outage: UNSAFE_TOUCHDOWN.
  Independent truth: touchdown XY error 28.733918 m; HOLD XY drift 0.640820 m.
- Runtime audit after the reporting fix: touchdown XY error 28.729 m, failed;
  dynamics, no bounce, disarm timing, and fresh zero outputs passed.
  Passing cleanup does not erase the position failure.

The original audit is retained alongside audit_after_reporting_fix.json.
The full controller CSV, aligned truth, manifests, harness ground evidence,
and losslessly compressed original PX4 ULog are included. The compressed ULog is split into numbered .part files to fit the publishing
connector. From the case directory, reconstruct and decompress it with:

    cat 02_47_27.ulg.gz.part* > 02_47_27.ulg.gz
    gzip -d 02_47_27.ulg.gz
    sha256sum 02_47_27.ulg

Its uncompressed SHA-256 is
7d99d122e44fe6c06e9d4d1702412e91d56ce6173e8a90accb6f61c411cf3897.
Truth is used after flight only. No physical aircraft was flown.

Earlier flight evidence is in ../2026-10-03. Larger earlier ULogs remain on the
test computer, identified by path and SHA-256 in their truth audits; compact
aligned traces are included without downsampling. Hashes in SHA256SUMS.json
describe the files actually published here.

To continue toward the requested readiness target, see the airframe,
sensor-model, wind/outage-envelope, and independent-aiding inputs requested in
../../docs/validation_summary.md.
