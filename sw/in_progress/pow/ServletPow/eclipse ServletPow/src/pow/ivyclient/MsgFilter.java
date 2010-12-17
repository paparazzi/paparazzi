package pow.ivyclient;
import java.util.HashSet;

/**
 * this class filters the message, thanks to their hash code
 * it is used because Paparazzi send periodic messages that are often the same
 * so we filter them to lighten the server load and the bandwidth of the udp channel
 * @author genin
 *
 */
public class MsgFilter {

	
	private HashSet<String> msgset;
	/**
	 * create a message filter
	 */
	public MsgFilter(){
		msgset = new HashSet<String>();
	}
	/**
	 * inform whether the message is new or not
	 * @param msg
	 * @return true if the message is a new one and was added to the filter
	 */
	public boolean isNew(String msg){
		boolean res = msgset.add(msg);
		return res;
	}
	/**
	 * empty the filter
	 */
	public void resetfilter(){
		msgset.clear();
	}
}
