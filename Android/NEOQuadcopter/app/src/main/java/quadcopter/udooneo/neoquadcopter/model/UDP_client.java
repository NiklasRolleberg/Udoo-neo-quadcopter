package quadcopter.udooneo.neoquadcopter.model;

/**
 * Created by Niklas on 2016-03-10.
 */
import java.io.*;
import java.net.*;
import java.util.ArrayList;
import java.util.Observable;

class UDP_client extends Observable
{
    DatagramSocket clientSocket;
    InetAddress IPAddress;
    int port;

    boolean stop = false;

    //buffers
    byte[] sendData;
    byte[] receiveData;

    ArrayList<String> received;

    public UDP_client(String ip, int portnr) throws Exception
    {
        clientSocket = new DatagramSocket();
        IPAddress = InetAddress.getByName(ip);
        port = portnr;

        sendData = new byte[1024];
        receiveData = new byte[1024];

        received = new ArrayList<String>();

        clientSocket.setSoTimeout(100000);

        new Thread() {
            public void run() {
                read();
            }
        }.start();

    }

    public void send(String toSend)
    {
        //System.out.println("udp-client send message: " + toSend);
        sendData = toSend.getBytes();
        DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, port);
        try {
            clientSocket.send(sendPacket);
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println(e.toString());
        }
    }

    private void read()
    {
        while(!stop)
        {
            try {
                DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
                clientSocket.receive(receivePacket);
                String modifiedSentence = new String(receivePacket.getData(),
                        receivePacket.getOffset(),receivePacket.getLength());
                //System.out.println("Received: " + modifiedSentence);
                received.add(modifiedSentence);
                setChanged();
                notifyObservers();

            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public ArrayList<String> getMessages() {
        return received;
    }

    public void close()
    {
        System.out.println("Closing udp client");
        stop = true;
        clientSocket.close();
    }
}
