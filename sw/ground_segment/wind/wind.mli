val wind : Geometry_2d.pt_2D -> Geometry_2d.pt_2D array -> float ->
  (Geometry_2d.pt_2D * float * float)
(** [wind wind_init speeds precision] returns the wind, air speed mean and std dev *)
