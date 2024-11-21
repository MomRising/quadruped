use std::sync::{Arc,Mutex};
use rclrs::RclrsError;
use std_msgs::msg::String as StringMsg;
use geometry_msgs::msg::Twist;
use cus_interfaces::msg::Base;
use std::time::Duration;
use std::thread;

struct RepuberNode {
    node:Arc<rclrs::Node>,
    publisher:Arc<rclrs::Publisher<StringMsg>>,
    _subscription:Arc<rclrs::Subscription<StringMsg>>,
    data:Arc<Mutex<Option<StringMsg>>>,
}

impl RepuberNode {
    fn new(context:&rclrs::Context) -> Result<Self,RclrsError> {
        let node = rclrs::create_node(context, "base_node")?;
        let data = Arc::new(Mutex::new(Some(StringMsg{data:String::from("hello")})));
        let data_cb = Arc::clone(&data);
        let publisher = node.create_publisher("out_topic", rclrs::QOS_PROFILE_DEFAULT)?;
        let _subscription = node
            .create_subscription(
            "in_topic",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg:StringMsg| {*data_cb.lock().unwrap() = Some(msg)},
            )?;

        Ok(Self {
            node,
            publisher,
            _subscription,
            data,
        })
    }

    fn republish(&self) -> Result<(),RclrsError> {
        if let Some(msg) = &*self.data.lock().unwrap() {
            self.publisher.publish(msg)?;
        };
        Ok(())
    }
}

fn main() -> Result<(),rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(RepuberNode::new(&context)?);
    let republisher_cl = Arc::clone(&republisher);
    thread::spawn(move || -> Result<(),RclrsError>{
        loop {
            thread::sleep(Duration::from_secs(1));
            republisher_cl.republish()?;
        }
    });

    rclrs::spin(Arc::clone(&republisher.node))
}