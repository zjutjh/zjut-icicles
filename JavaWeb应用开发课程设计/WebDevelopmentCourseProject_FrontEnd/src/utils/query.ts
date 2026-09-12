export function compactObject<T extends object>(input: T) {
  return Object.fromEntries(
    Object.entries(input as Record<string, unknown>).filter(([, value]) => value !== undefined && value !== null && value !== ''),
  ) as Partial<T>
}
