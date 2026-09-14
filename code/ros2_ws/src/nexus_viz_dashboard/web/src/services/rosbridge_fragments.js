// rosbridge may fragment a full-resolution Image even when the preview is JPEG.
export function createEnvelopeDecoder(clock = () => performance.now()) {
  const pending = new Map();
  return text => {
    let message;
    try { message = JSON.parse(text); } catch { return null; }
    if (message?.op !== "fragment") return message;
    const { id, num, total, data } = message, now = clock();
    for (const [key, value] of pending) if (now - value.started > 10000) pending.delete(key);
    if (typeof id !== "string" || !Number.isInteger(total) || total < 1 || total > 32
        || !Number.isInteger(num) || num < 0 || num >= total || typeof data !== "string") return null;
    if (!pending.has(id)) {
      if (pending.size >= 2) pending.delete(pending.keys().next().value);
      pending.set(id, { started: now, total, parts: new Map(), length: 0 });
    }
    const frame = pending.get(id);
    if (frame.total !== total) { pending.delete(id); return null; }
    if (!frame.parts.has(num)) { frame.parts.set(num, data); frame.length += data.length; }
    if (frame.length > 16 * 1024 * 1024) { pending.delete(id); return null; }
    if (frame.parts.size !== total) return null;
    pending.delete(id);
    try { return JSON.parse(Array.from({ length: total }, (_, index) => frame.parts.get(index)).join("")); }
    catch { return null; }
  };
}
