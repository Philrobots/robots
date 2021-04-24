import { enumFactory } from 'node-factory';
import { Corner } from '@/types/corner';

export const CornerFactory = enumFactory<Corner>(Object.values(Corner));
