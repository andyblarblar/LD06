use LDO6::ld06_driver::LD06;

fn main() {
    simple_log::quick!();
    
    let mut ld = LD06::new_auto_port().unwrap();
    
    ld.listen();
    
}
