window.SIDEBAR_ITEMS = {"fn":[["crate_name_from_path","Returns the crate name that is derived from a crate object file path."],["get_containing_crate_name","Parses the given symbol string to try to find the name of the parent crate that contains the symbol.  Depending on the symbol, there may be multiple potential parent crates; if so, they are returned in order of likeliness:  the first crate name in the symbol is most likely to contain it. If the parent crate cannot be determined (e.g., a `no_mangle` symbol), then an empty `Vec` is returned."],["get_containing_crate_name_ranges","Same as `get_containing_crate_name()`, but returns the substring `Range`s of where the parent crate names  are located in the given `demangled_full_symbol` string."],["is_valid_crate_name_char","Crate names must be only alphanumeric characters, an underscore, or a dash."],["replace_containing_crate_name","Replaces the `old_crate_name` substring in the given `demangled_full_symbol` with the given `new_crate_name`,  if it can be found, and if the parent crate name matches the `old_crate_name`.  If the parent crate name can be found but does not match the expected `old_crate_name`, then None is returned."]]};