import { enumFactory } from 'node-factory';
import { Color } from '@/types/color';

export const ColorFactory = enumFactory<Color>(Object.values(Color));
