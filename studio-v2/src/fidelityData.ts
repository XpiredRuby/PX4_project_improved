import raw from '../../config/fidelity/x500_envelope.json';
import type { Envelope } from './fidelity';

export const X500_ENVELOPE = raw as unknown as Envelope;
