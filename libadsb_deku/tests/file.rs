use adsb_deku::{
    cpr::{self, Position},
    Frame,
};
use test_log::test;

const TEST_STR: &str = include_str!("../tests/lax-messages.txt");

#[test]
fn lax_messages() {
    // Read from test file and assert display implemented and non panic decode
    for line in TEST_STR.lines() {
        let len = line.chars().count();
        let hex = &mut line.to_string()[1..len - 1].to_string();
        let bytes = hex::decode(&hex).unwrap();
        // test non panic decode
        println!("{:02x?}", bytes);
        let frame = Frame::from_bytes(&bytes).unwrap();
        // test fmt::Display implemented
        assert_ne!("{}", format!("{frame}"));
    }
}

#[test]
fn test_surface_position() {
    let bytes = hex::decode("8c8013af3dc9e656539852a9618e").unwrap();
    let frame = Frame::from_bytes(&bytes).unwrap();
    match frame.df {
        adsb_deku::DF::ADSB(a) => match a.me {
            adsb_deku::adsb::ME::SurfacePosition(surface_position) => {
                assert_eq!(
                    cpr::surface_position_with_reference(
                        &surface_position,
                        (26.828576952763022, 75.80601239775218)
                    ),
                    Some(Position { latitude: 26.823504173149495, longitude: 75.80336644099309 })
                );
                assert_eq!(surface_position.lat_cpr, 76585);
                assert_eq!(surface_position.lon_cpr, 104530);
            }
            x => {
                panic!("Unexpected message: {:?}", x)
            }
        },
        x => {
            panic!("Unexpected message: {:?}", x)
        }
    }
}
