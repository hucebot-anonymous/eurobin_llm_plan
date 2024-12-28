interface root {
    pattern_type: PatternType;
    object: string;
    initial_kitchen: Kitchen;
    initial_location: Location;
    target_kitchen: Kitchen;
    target_location: Location;
    chain_of_thought: string;
  }

  enum Location {
    dishwasher ="Dishwasher",
    table  = "Table",
    person = "Person",
    cabinet = "Cabinet"
  }

  enum Kitchen {
    inria = "Inria",
    dlr  = "DLR",
    kit = "KIT",
  }

  enum PatternType {
    a = "A",
    b = "B"
  }