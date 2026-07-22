const asError = value => value instanceof Error
  ? value
  : new Error(String(value ?? 'Worker task failed'));

export const createAbortError = (message = 'Computation cancelled') => {
  const error = new Error(message);
  error.name = 'AbortError';
  return error;
};

export const isAbortError = error => error instanceof Error && error.name === 'AbortError';

/**
 * Run one task on a dedicated worker. Terminating the worker is the only
 * reliable way to cancel a synchronous WebAssembly call while it is running.
 */
export const createCancelableWorkerTask = ({ worker, id, kind, payload }) => {
  let settled = false;
  let rejectTask = null;

  const terminate = () => {
    worker.onmessage = null;
    worker.onerror = null;
    worker.terminate();
  };

  const promise = new Promise((resolve, reject) => {
    rejectTask = reject;

    worker.onmessage = event => {
      const message = event.data || {};
      if (message.id !== id || settled) return;
      settled = true;
      terminate();
      if (message.ok) {
        resolve(message.result);
      } else {
        reject(new Error(message.error || 'Worker task failed'));
      }
    };

    worker.onerror = event => {
      if (settled) return;
      settled = true;
      terminate();
      reject(asError(event?.error || event?.message || 'Compute worker error'));
    };

    try {
      worker.postMessage({ id, kind, payload });
    } catch (error) {
      settled = true;
      terminate();
      reject(asError(error));
    }
  });

  return {
    promise,
    cancel(message = 'Computation cancelled') {
      if (settled) return false;
      settled = true;
      terminate();
      rejectTask(createAbortError(message));
      return true;
    }
  };
};
