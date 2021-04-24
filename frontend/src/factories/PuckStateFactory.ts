import { enumFactory } from 'node-factory';
import { PuckState } from '@/types/puckState';

export const PuckStateFactory = enumFactory<PuckState>(
  Object.values(PuckState)
);
