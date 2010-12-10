package pow;
import java.util.HashSet;

/**
 * classe permettant de filtrer les messages waypoints inutile
 * @author genin
 *
 */
public class MsgFilter {

	
	private HashSet<String> msgset;
	
	public MsgFilter(){
		msgset = new HashSet<String>();
	}
	/**
	 * informe si le message est deja contenu dans le filtre
	 * et l'ajoute le cas echeant
	 * @param msg
	 * @return
	 */
	public boolean isNew(String msg){
		boolean res = msgset.add(msg);
		return res;
	}
	/**
	 * vide le filtre
	 */
	public void resetfilter(){
		msgset.clear();
	}
}
