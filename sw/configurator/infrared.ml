let create_sheet = fun sheets xml ->
  let airframe = Frame.create sheets in
  Notebook.create_sheet sheets "Infrared" airframe
