use prusti_contracts::*;

#[allow(unused_imports)] // for prusti spec
use volatile::Volatile;


#[extern_spec]
impl<T: Copy> Volatile<T>{
    #[pure]
    pub fn read(&self) -> T;

    #[ensures(self.read() === val)]
    pub fn write(&mut self, val: T);
}
