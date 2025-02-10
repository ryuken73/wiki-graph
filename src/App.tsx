import React from 'react';
import styled from 'styled-components';
// import {ForceGraph2D} from 'react-force-graph';
import Graph2D from './Graph2D';
import Backdrop from './Components/Backdrop';
import {
  getBacklinksByContentId,
  getForwardlinksByBacklinkId,
  getNodeByContentId,
  getNodeByBacklinkId,
} from './js/serverApi.js';
import {
  expandNetworkData,
  addNewNodeNExpandNetworkData
} from './js/dataHandlers.js';
import {
  isLastLeafNode,
  notInNodes,
  getNodeIdsConnected, 
  getLinksOfNode,
  removeNodes,
  getNodeTextById,
  getNumberOfNodes,
  genFocusNode
} from './js/graphHandlers.js';
import './App.css'
import ExpandedContainer from './Components/ExpandedContainer';
import ForwardlinkContainer from './Components/ForwardlinkContainer.jsx';
import BacklinkContainer from './Components/BacklinkContainer.jsx';
import AutoComplete from './Components/AutoComplete.jsx';
import NodeHandler from './Components/NodeHandler';

const Container = styled.div``
const AbsoluteBoxForSearch = styled.div`
  position: absolute;
  top: 5vh;
  left: 5%;
`
const AbsoluteBoxForNodesShown = styled.div`
  display: grid;
  /* grid-template-columns: 1fr 1fr 1fr; */
  grid-template-columns: 1fr;
  grid-gap: 5px;
  position:  absolute;
  top: 15vh;
  right: 5%;
  z-index: 10;  
  /* background: rgba(255,255,255,0.1); */
  /* height: 80%; */
  padding: 5px;
  border: 2px solid rgba(255, 255, 255, 0.1);
`
const ActiveExpandedNode = styled.div`
  grid-column: 1/-1;
  color: yellow;
  font-weight: 200;
  background: rgba(255,255,255,0.1);
  padding: 3px; 
  font-size: 20px;
`
const NodeText = styled.span``
const TotalNodes = styled.span`
`

const COLORS = {
  person: 'yellow',
  other: 'grey'
}
const contentId = '가수_한국_C_004301_유재석';
const initialNode = {
  id:  contentId,
  text: '유재석',
  color: COLORS.person,
  value: 100
}

function App() {
  // const [lastNetworkData, setLastNetworkData] = React.useState({nodes:[initialNode], links:[]});
  // const [nodesExpanded, setNodesExpanded] = React.useState([initialNode]);
  const [lastNetworkData, setLastNetworkData] = React.useState({nodes:[], links:[]});
  const [nodesExpanded, setNodesExpanded] = React.useState([]);
  const [activeExpandedNodeId, setActiveExpandedNodeId] = React.useState(null);
  const [backlinksToShow, setBacklinksToShow] = React.useState([]);
  const [forwardlinksToShow, setForwardlinkToShow] = React.useState([]);
  const [showBackdrop, setShowBackdrop] = React.useState(false);
  const [checkedNodeList, setCheckedNodeList] = React.useState([]);
  const graphRef = React.useRef(null);
  const focusNode2D = genFocusNode(graphRef, '2D');

  console.log(checkedNodeList)

  const addCheckedNodeList = React.useCallback((nodeId) => {
    const targetNode = nodesExpanded.find(node => node.id === nodeId);
    setCheckedNodeList((checkedNodeList) => {
      return [
        ...checkedNodeList,
        targetNode
      ];
    })
  }, [nodesExpanded]);
  const delCheckedNodeList = React.useCallback((nodeId) => {
    setCheckedNodeList((checkedNodeList) => {
      const newCheckedNodeList = [...checkedNodeList];
      return newCheckedNodeList.filter(node => node.id !== nodeId)
    })
  }, []);
  const postExpandTask = React.useCallback((node) => {
    setNodesExpanded((nodes) => {
      const isDup = nodes.some(existingNode => existingNode.id === node.id);
      return isDup ? nodes : [node, ...nodes]
    })
    setActiveExpandedNodeId(node.id);
  }, [])

  const addNewNode = React.useCallback(async (nodeId, isNodeContent) => {
    setShowBackdrop(true)
    const nodeInfo = isNodeContent ? await getNodeByContentId(nodeId) : await getNodeByBacklinkId(nodeId);
    const expandNodes = isNodeContent ? await getBacklinksByContentId(nodeId) : await getForwardlinksByBacklinkId(nodeId)
    const newNode = nodeInfo[0];
    const includeOnlyContents = true;
    setLastNetworkData((lastNetworkData) => {
      const isForwarding = !isNodeContent;
      const newNetworkData = addNewNodeNExpandNetworkData(newNode, expandNodes, lastNetworkData, includeOnlyContents, isForwarding);
      const addedNode = newNetworkData.nodes.find(node => node.id === nodeId);
      postExpandTask(addedNode)
      setShowBackdrop(false)
      return newNetworkData;
    })
  }, [postExpandTask]);

  // get initial network data
  React.useEffect( () => {
    const IS_NODE_CONTENT = true;
    // addNewNode(contentId, IS_NODE_CONTENT);
  }, [addNewNode])

  const expandNode = React.useCallback(async (node, isForwardlink=false) => {
    console.log('node click:', node)
    setShowBackdrop(true)
    const {id, backlinkId, isContent} = node;
    console.log(isContent)
    if(!isContent){
      setShowBackdrop(false)
      return false
    }
    console.log(node);
    const includeOnlyContents = true;
    const rows = isForwardlink ?
      await getForwardlinksByBacklinkId(backlinkId) :
      await getBacklinksByContentId(id)
    setLastNetworkData(lastNetworkData => {
      return expandNetworkData(rows, node.id, lastNetworkData, includeOnlyContents, isForwardlink);
    })
    postExpandTask(node);
    setShowBackdrop(false)
    // focusNode2D(node)
  }, [postExpandTask]);

  const removeNode = React.useCallback((eventOrId, hiddenParam) => {
    const fromEvent = eventOrId && eventOrId.nativeEvent; 
    const centerNodeId = fromEvent ? eventOrId.target.id : eventOrId ;
    const keepCenterNode = hiddenParam
    const KEEP_EXPANDED_BUT_NO_LEAF_NODE = true;
    console.log('removeNode centerNodeId:', centerNodeId, keepCenterNode)
    setLastNetworkData((lastNetworkData) => {
      const nodeIdsConnected = getNodeIdsConnected(lastNetworkData, centerNodeId);
      const lastLeafNodeIds = nodeIdsConnected.filter((nodeId) => {
        return isLastLeafNode(lastNetworkData, centerNodeId, nodeId)
      })
      // const withCenterNode = [centerNodeId, ...lastLeafNodeIds];
      const nodeIdsToDeleteFiltered = KEEP_EXPANDED_BUT_NO_LEAF_NODE ?
        [...(lastLeafNodeIds.filter(nodeId => notInNodes(nodesExpanded, nodeId)))]:
        [...lastLeafNodeIds]
      const nodeIdsToDelete = keepCenterNode ? nodeIdsToDeleteFiltered: [centerNodeId, ...nodeIdsToDeleteFiltered];

      console.log(nodeIdsToDelete)
      const newNodes = [...lastNetworkData.nodes].filter(node => {
        return !nodeIdsToDelete.includes(node.id)
      })
      const newLinks = [...lastNetworkData.links].filter(link => {
        return !nodeIdsToDelete.includes(link.source.id) && !nodeIdsToDelete.includes(link.target.id);
      })
      return {
        nodes: newNodes,
        links: newLinks
      };
    })
    setNodesExpanded(nodesExpanded => {
      return [...nodesExpanded].filter(node => {
        return node.id !== centerNodeId;
      })
    })
  }, [nodesExpanded])

  console.log('lastNetwokData=', lastNetworkData)
  const totalNodes = getNumberOfNodes(lastNetworkData);
  const activeExpandedText = getNodeTextById(lastNetworkData, activeExpandedNodeId);

  return (
    <Container>
      <Backdrop
        open={showBackdrop}
        setOpen={setShowBackdrop}
      ></Backdrop>
      <NodeHandler
        checkedNodeList={checkedNodeList}
        setCheckedNodeList={setCheckedNodeList}
        removeNode={removeNode}
        expandNode={expandNode}
      ></NodeHandler>
      <Graph2D
        ref={graphRef}
        graphData={lastNetworkData}
        expandNode={expandNode}
      ></Graph2D>
      <AbsoluteBoxForSearch>
        <AutoComplete
          onSuggestSelected={addNewNode}
        ></AutoComplete>
      </AbsoluteBoxForSearch>
      <AbsoluteBoxForNodesShown>
        <ActiveExpandedNode>
          {activeExpandedNodeId !== null ? (
            <NodeText>{activeExpandedText}</NodeText>
          ) : (
            <TotalNodes 
            >[{totalNodes}]</TotalNodes>
          )}
        </ActiveExpandedNode>
        {/* <BacklinkContainer
          lastNetworkData={lastNetworkData}
          expandNode={expandNode}
          activeExpandedNodeId={activeExpandedNodeId}
          backlinksToShow={backlinksToShow}
        ></BacklinkContainer> */}
        <ExpandedContainer
          nodesExpanded={nodesExpanded}
          removeNode={removeNode}
          lastNetworkData={lastNetworkData}
          setLastNetworkData={setLastNetworkData}
          setBacklinksToShow={setBacklinksToShow}
          setForwardlinksToShow={setForwardlinkToShow}
          setActiveExpandedNodeId={setActiveExpandedNodeId}
          addCheckedNodeList={addCheckedNodeList}
          delCheckedNodeList={delCheckedNodeList}
          focusNode={focusNode2D}
        
        ></ExpandedContainer>
        {/* <ForwardlinkContainer
          lastNetworkData={lastNetworkData}
          expandNode={expandNode}
          activeExpandedNodeId={activeExpandedNodeId}
          forwardlinksToShow={forwardlinksToShow}
        ></ForwardlinkContainer> */}
      </AbsoluteBoxForNodesShown>
    </Container>
  )
}

export default App
