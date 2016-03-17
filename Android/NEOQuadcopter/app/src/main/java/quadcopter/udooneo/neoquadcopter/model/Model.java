package quadcopter.udooneo.neoquadcopter.model;

import java.util.ArrayList;
import java.util.Observable;
import java.util.Observer;

/**
 * Created by Niklas on 2016-03-10.
 */
public class Model implements Observer {

    UDP_client udp_client = null;

    public Model() {
        System.out.println("Model created");
    }

    public boolean setupConnection(String ip, int port) {
        try {
            udp_client = new UDP_client(ip,port);
            udp_client.addObserver(this);
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public void closeConnection() {
        System.out.println("Closing connection");
        udp_client.close();
        udp_client=null;
    }


    public void send(final String s) {
        System.out.println("Sending: "+s);

        if(udp_client != null) {

            new Thread() {
                public void run() {
                    udp_client.send(s);
                }
            }.start();
        }
    }

    @Override
    public void update(Observable observable, Object data) {
        //System.out.println("Something changed");

        ArrayList<String> newMessages = udp_client.getMessages();

        while (!newMessages.isEmpty()) {
            String s = newMessages.get(0);
            processMessage(s);
            newMessages.remove(0);
        }

    }

    private void processMessage(String s) {
        System.out.println("Received: " + s);
    }
}
