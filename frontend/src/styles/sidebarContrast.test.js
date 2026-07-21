import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';

const stylesheet = readFileSync(resolve(process.cwd(), 'src/index.css'), 'utf8');

const readColorToken = (name) => {
  const match = stylesheet.match(new RegExp(`--${name}:\\s*(#[0-9a-fA-F]{6})`));
  if (!match) throw new Error(`Missing CSS color token: --${name}`);
  return match[1];
};

const relativeLuminance = (hexColor) => {
  const channels = hexColor
    .slice(1)
    .match(/.{2}/g)
    .map(channel => parseInt(channel, 16) / 255)
    .map(channel => (
      channel <= 0.04045
        ? channel / 12.92
        : ((channel + 0.055) / 1.055) ** 2.4
    ));

  return (0.2126 * channels[0]) + (0.7152 * channels[1]) + (0.0722 * channels[2]);
};

const contrastRatio = (foreground, background) => {
  const foregroundLuminance = relativeLuminance(foreground);
  const backgroundLuminance = relativeLuminance(background);
  const lighter = Math.max(foregroundLuminance, backgroundLuminance);
  const darker = Math.min(foregroundLuminance, backgroundLuminance);
  return (lighter + 0.05) / (darker + 0.05);
};

describe('sidebar typography tokens', () => {
  it.each(['text', 'text-2', 'text-3'])('%s is true white for maximum visibility', token => {
    expect(readColorToken(token).toLowerCase()).toBe('#ffffff');
  });

  it.each(['text', 'text-2', 'text-3'])('%s meets WCAG AA contrast on sidebar surfaces', token => {
    const foreground = readColorToken(token);

    expect(contrastRatio(foreground, readColorToken('panel'))).toBeGreaterThanOrEqual(4.5);
    expect(contrastRatio(foreground, readColorToken('surface'))).toBeGreaterThanOrEqual(4.5);
  });

  it('uses the shared native font stacks for UI and equation text', () => {
    expect(stylesheet).toMatch(/--font-sans:\s*ui-sans-serif/);
    expect(stylesheet).toMatch(/--font-mono:\s*ui-monospace/);
    expect(stylesheet).toMatch(/body\s*{[^}]*font-family:\s*var\(--font-sans\)/s);
    expect(stylesheet).toMatch(/\.eq-line\s*{[^}]*font-family:\s*var\(--font-mono\)/s);
    expect(stylesheet).toMatch(/\.eq-line \.sym,[^}]*color:\s*var\(--text\)/s);
    expect(stylesheet).toMatch(/\.eq-custom-input\s*{[^}]*color:\s*var\(--text\)/s);
  });
});
