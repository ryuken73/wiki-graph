import React from 'react';
import styled from 'styled-components';
import {getBacklinksByContentId} from '../js/serverApi.js';
import {
  mkNetworkData, 
  extractBacklinks,
  extractForwardlinks
} from '../js/dataHandlers.js';
import {
  RowContainer,
  Node,
  Title,
  DelButton,
  TimeStamp
} from './NodeStyles.js'

function NodeExpanded(props) {
  const {
    node, 
    removeNode, 
    lastNetworkData,
    setLastNetworkData, 
    setBacklinksToShow,
    setForwardlinksToShow,
    setActiveExpandedNodeId,
    focusNode
  } = props;
  const [isShowAll, setIsShowAll] = React.useState(false);
  const onClickTitle = React.useCallback((event) => {
    const id = event.target.id;
    const clickedNode = lastNetworkData.nodes.find(node => node.id === id);
    setActiveExpandedNodeId(id);
    const backlinkNodes = extractBacklinks(lastNetworkData, id);
    setBacklinksToShow(backlinkNodes)
    console.log('backlinkNodes', backlinkNodes)
    const forwardlinkNodes = extractForwardlinks(lastNetworkData, id);
    setForwardlinksToShow(forwardlinkNodes)
    console.log('forwardlinkNodes', forwardlinkNodes)
    focusNode(clickedNode)
  }, [lastNetworkData])
  const toggleShowsAllBacklinks = React.useCallback((event) => {
    const {id} = event.target;
    setIsShowAll( async (isShowAll) => {
      const includeOnlyContents = isShowAll;
      const rows = await getBacklinksByContentId(id)
      setLastNetworkData((lastNetworkData) => {
        const newNetworkData = mkNetworkData(rows, id, lastNetworkData, includeOnlyContents);
        return newNetworkData
      })
    })
  }, [])
  return (
    <RowContainer>
      <Node key={node.id}>
        <Title
          id={node.id}
          onClick={onClickTitle}
        >
          {node.text}
        </Title>
        <DelButton id={node.id} onClick={removeNode}>
          [Del]
        </DelButton>
      </Node>
      <TimeStamp
        onClick={toggleShowsAllBacklinks}
        id={node.id}
      >
        backlinks [{node.backlinkCount}]
      </TimeStamp>
    </RowContainer>
  )
}

export default React.memo(NodeExpanded)