import { describe, expect, it } from 'vitest';
import { describeBasinComputationError } from './basinError';

describe('describeBasinComputationError', () => {
  it('turns opaque WebAssembly traps into actionable guidance', () => {
    expect(describeBasinComputationError(new WebAssembly.RuntimeError('unreachable')))
      .toMatch(/WebAssembly resources/);
    expect(describeBasinComputationError(new Error('memory access out of bounds')))
      .toMatch(/before convergence/i);
  });

  it('preserves meaningful backend validation errors', () => {
    expect(describeBasinComputationError(new Error('Target tube is empty')))
      .toBe('Target tube is empty');
  });
});
