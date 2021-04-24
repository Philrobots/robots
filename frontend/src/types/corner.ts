export enum Corner {
  UNSET = 'X',
  A = 'A',
  B = 'B',
  C = 'C',
  D = 'D',
}

// TODO : Couldn't this be moved in the enum?
export const getNextCorner = (corner: Corner): Corner => {
  switch (corner) {
    case Corner.A:
      return Corner.B;
    case Corner.B:
      return Corner.C;
    case Corner.C:
      return Corner.D;
    case Corner.D:
    default:
      return Corner.A;
  }
};
