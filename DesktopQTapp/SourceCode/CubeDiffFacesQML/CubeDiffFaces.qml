import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D

Rectangle {
    id: root
    width: 220
    height: 220
    visible: true
    color: "black"

    Node {
        id: standAloneScene
        DirectionalLight { ambientColor: Qt.rgba(1.0, 1.0, 1.0, 1.0) }
        Node {
            id: node
            Repeater3D {
                model: [
                    ["Dice1.svg", 0, 0 ],
                    ["Dice6.svg", 0, 180 ],
                    ["Dice3.svg", 0, 90 ],
                    ["Dice4.svg", 0, -90 ],
                    ["Dice2.svg", 90, 0 ],
                    ["Dice5.svg", -90, 0 ],
                ]
                delegate: Node {
                    eulerRotation.x: modelData[1]
                    eulerRotation.y: modelData[2]
                    Model {
                        source: "#Rectangle"
                        materials: [
                            DefaultMaterial {
                                diffuseMap: Texture {
                                    sourceItem: Item {
                                        anchors.centerIn: parent
                                        width: 234
                                        height: 234
                                        Image {
                                            anchors.fill: parent
                                            source: modelData[0]
                                            sourceSize: Qt.size(width, height)
                                            cache: false
                                        }
                                    }
                                }
                            }
                        ]
                        z: 50
                    }
                }
//                eulerRotation.x: 40
//                eulerRotation.y: 20
//                eulerRotation.z: 0
            }
        }
        OrthographicCamera {
            id: cameraOrthographicFront
            lookAtNode: node
            x: 0; y: 0; z: 100
            //property double sc: 300/Math.max(Math.min(page.width, page.height), 1)
            scale: Qt.vector3d(1, 2, 1)
        }

        scale: Qt.vector3d(1.3,1.3,1.3)
    }

    View3D {
        anchors.fill: parent
        importScene: standAloneScene
        camera: cameraOrthographicFront
    }

//    NumberAnimation {
//        target: node
//        property: "eulerRotation.y"
//        loops: Animation.Infinite
//        running: true
//        from: 720; to: 0
//        duration: 10000
//    }
//    NumberAnimation {
//        target: node
//        property: "eulerRotation.x"
//        loops: Animation.Infinite
//        running: true
//        from: 360; to: 0
//        duration: 10000
//    }



    function updateCubeOrientation(yaw: QVariant,pitch: QVariant,roll: QVariant){
      node.eulerRotation.x = pitch
      node.eulerRotation.y = yaw
      node.eulerRotation.z = roll
    }
}
