/** 浏览器端消息的最小契约；前端只展示，不执行坐标转换、融合或指标计算。 */
export const INPUT_MODES = Object.freeze({ LIVE: "LIVE", REPLAY: "REPLAY", NO_INPUT: "NO_INPUT" });
export const SOURCE_MODES = Object.freeze({ UWB: "UWB", VISION: "VISION", FUSED: "FUSED", UNKNOWN: "UNKNOWN" });
export const EMPTY_METRICS = Object.freeze({ rmse: null, p95: null, max: null, availability: null, samples: null, runId: null });

export function finiteOrNull(value) {
  if (value === null || value === undefined || value === "") return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

export function formatNumber(value, digits = 2, fallback = "—") {
  const number = finiteOrNull(value);
  return number === null ? fallback : number.toFixed(digits);
}
