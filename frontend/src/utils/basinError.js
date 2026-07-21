const WASM_RESOURCE_TRAP = /(?:^|\b)(?:unreachable|out of memory|allocation failed|memory access out of bounds)(?:\b|$)/i;

export const describeBasinComputationError = error => {
  const message = error instanceof Error ? error.message : String(error ?? 'Unknown basin error');
  if (WASM_RESOURCE_TRAP.test(message)) {
    return 'The basin computation exceeded the available WebAssembly resources. Reduce the search depth or narrow the position domain. If the failure persists, report the current a, b, ε, and axis bounds.';
  }
  return message;
};
