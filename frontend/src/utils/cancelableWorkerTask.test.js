import { describe, expect, it, vi } from 'vitest';
import { createCancelableWorkerTask, isAbortError } from './cancelableWorkerTask';

const createWorker = () => ({
  onmessage: null,
  onerror: null,
  postMessage: vi.fn(),
  terminate: vi.fn()
});

describe('createCancelableWorkerTask', () => {
  it('resolves the matching worker response and releases the worker', async () => {
    const worker = createWorker();
    const task = createCancelableWorkerTask({ worker, id: 7, kind: 'compute', payload: { value: 2 } });
    expect(worker.postMessage).toHaveBeenCalledWith({ id: 7, kind: 'compute', payload: { value: 2 } });

    worker.onmessage({ data: { id: 7, ok: true, result: 4 } });

    await expect(task.promise).resolves.toBe(4);
    expect(worker.terminate).toHaveBeenCalledOnce();
  });

  it('terminates immediately and rejects with an abort error when cancelled', async () => {
    const worker = createWorker();
    const task = createCancelableWorkerTask({ worker, id: 8, kind: 'compute', payload: {} });

    expect(task.cancel('Basin computation cancelled')).toBe(true);

    const error = await task.promise.catch(value => value);
    expect(isAbortError(error)).toBe(true);
    expect(worker.terminate).toHaveBeenCalledOnce();
    expect(task.cancel()).toBe(false);
  });
});
